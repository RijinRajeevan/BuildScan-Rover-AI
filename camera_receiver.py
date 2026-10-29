"""
BuildScan Rover — Camera Receiver Module
═════════════════════════════════════════
Handles robust MJPEG stream reception from the Raspberry Pi 4 camera
(Logitech Brio 100 on /dev/video0), supporting both:
  1. The custom stream server (http://<IP>:8080/video, health at /health)
  2. The ROS 2 web_video_server (http://<IP>:8080/stream?topic=/image_raw)

Features:
- Configurable via BUILDSCAN_PI_IP, BUILDSCAN_PI_PORT, BUILDSCAN_CAMERA_URL
- Proactive health endpoint check (GET /health)
- Non-blocking latest-frame buffer (no unbounded backlog or latency buildup)
- Automatic reconnection with exponential backoff on Pi restart / network drop
- Per-frame acquisition timestamp
- Thread-safe singleton lifecycle
"""

import os
import sys
import time
import json
import logging
import threading
import urllib.request
import urllib.error
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("buildscan.camera")

# Default environment configuration
DEFAULT_PI_IP = "10.21.210.129"
DEFAULT_PI_PORT = 8080
DEFAULT_MJPEG_PATH = "/video"
DEFAULT_ROS_STREAM_PATH = "/stream?topic=/image_raw"


def get_default_config() -> Dict[str, Any]:
    """Resolves camera endpoint configuration from environment variables."""
    ip = os.getenv("BUILDSCAN_PI_IP", DEFAULT_PI_IP).strip()
    try:
        port = int(os.getenv("BUILDSCAN_PI_PORT", str(DEFAULT_PI_PORT)))
    except ValueError:
        port = DEFAULT_PI_PORT
    url_override = os.getenv("BUILDSCAN_CAMERA_URL", "").strip()
    return {
        "ip": ip,
        "port": port,
        "url_override": url_override or None,
    }


def check_pi_health(ip: str = None, port: int = None, timeout: float = 2.5) -> Dict[str, Any]:
    """
    Checks the Pi camera health endpoint (GET /health).
    Expected response from brio_stream.py: {"camera_open": true, "device": "/dev/video0"}
    """
    cfg = get_default_config()
    target_ip = ip or cfg["ip"]
    target_port = port or cfg["port"]
    health_url = f"http://{target_ip}:{target_port}/health"

    result = {
        "reachable": False,
        "camera_open": False,
        "device": None,
        "url": health_url,
        "error": None,
    }
    try:
        req = urllib.request.Request(
            health_url,
            headers={"User-Agent": "BuildScan-HealthCheck/1.0"}
        )
        with urllib.request.urlopen(req, timeout=timeout) as res:
            if res.status == 200:
                data = json.loads(res.read().decode("utf-8", errors="replace"))
                result["reachable"] = True
                result["camera_open"] = bool(data.get("camera_open", False))
                result["device"] = data.get("device") or data.get("camera")
            else:
                result["error"] = f"HTTP {res.status}"
    except urllib.error.URLError as e:
        result["error"] = f"Connection error: {e.reason}"
    except Exception as e:
        result["error"] = str(e)

    return result


@dataclass
class CameraStatus:
    connected: bool = False
    source_url: str = ""
    frames_received: int = 0
    fps: float = 0.0
    last_frame_ts: float = 0.0
    last_frame_iso: str = ""
    error_message: str = ""
    reconnect_count: int = 0
    health_info: Optional[Dict[str, Any]] = None


class PiCameraReceiver:
    """
    Thread-safe camera receiver that pulls MJPEG frames from the Pi without blocking
    the dashboard UI or accumulating old frames in memory.
    """

    def __init__(self, pi_ip: str = None, pi_port: int = None, stream_url: str = None):
        cfg = get_default_config()
        self.pi_ip = pi_ip or cfg["ip"]
        self.pi_port = pi_port or cfg["port"]
        self.url_override = stream_url or cfg["url_override"]

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._last_frame: Optional[np.ndarray] = None
        self._last_jpeg: Optional[bytes] = None
        self._status = CameraStatus()

    def get_stream_url(self) -> str:
        if self.url_override:
            return self.url_override
        return f"http://{self.pi_ip}:{self.pi_port}{DEFAULT_MJPEG_PATH}"

    def get_fallback_stream_url(self) -> str:
        return f"http://{self.pi_ip}:{self.pi_port}{DEFAULT_ROS_STREAM_PATH}"

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(
                target=self._worker_loop,
                daemon=True,
                name="bscan-cam-rx"
            )
            self._thread.start()
            log.info(f"Camera receiver thread started for {self.get_stream_url()}")

    def stop(self):
        with self._lock:
            self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        log.info("Camera receiver thread stopped")

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], CameraStatus]:
        """Returns the most recently decoded frame and current status with freshness validation."""
        with self._lock:
            now = time.time()
            # If no frame received within 2.0s, or Pi health explicitly reports camera_open=False, mark disconnected
            is_stale = (now - self._status.last_frame_ts) > 2.0 if self._status.last_frame_ts > 0 else True
            hw_closed = self._status.health_info is not None and not self._status.health_info.get("camera_open", True)

            if is_stale or hw_closed:
                self._status.connected = False
                if hw_closed:
                    self._status.error_message = "Pi camera hardware is closed (/dev/video0 busy or unplugged)"
                elif is_stale and self._status.last_frame_ts > 0:
                    self._status.error_message = f"Stream stalled (no new frame for {now - self._status.last_frame_ts:.1f}s)"

            frame = None if (self._last_frame is None or not self._status.connected) else self._last_frame.copy()
            status = CameraStatus(
                connected=self._status.connected,
                source_url=self._status.source_url,
                frames_received=self._status.frames_received,
                fps=self._status.fps if self._status.connected else 0.0,
                last_frame_ts=self._status.last_frame_ts,
                last_frame_iso=self._status.last_frame_iso,
                error_message=self._status.error_message,
                reconnect_count=self._status.reconnect_count,
                health_info=self._status.health_info,
            )
            return frame, status

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._last_jpeg

    def _worker_loop(self):
        """Worker loop that handles stream consumption and auto-recovery."""
        active_url = self.get_stream_url()
        tried_fallback = False
        backoff = 1.0

        fps_t0 = time.time()
        fps_count = 0

        while self._running:
            # Check Pi health first
            health = check_pi_health(self.pi_ip, self.pi_port, timeout=1.5)
            with self._lock:
                self._status.health_info = health

            if health["reachable"] and not health.get("camera_open", False):
                with self._lock:
                    self._status.connected = False
                    self._status.error_message = f"Pi Camera hardware is closed ({health.get('device', '/dev/video0')} unavailable)"
                    self._last_frame = None
                    self._status.fps = 0.0
                time.sleep(1.0)
                continue

            try:
                log.info(f"Connecting to Pi camera stream: {active_url}")
                req = urllib.request.Request(
                    active_url,
                    headers={
                        "User-Agent": "BuildScan-Rover-Receiver/1.0",
                        "Accept": "multipart/x-mixed-replace, image/jpeg, */*",
                    }
                )
                with urllib.request.urlopen(req, timeout=3.0) as resp:
                    with self._lock:
                        self._status.connected = True
                        self._status.source_url = active_url
                        self._status.error_message = ""
                    backoff = 1.0
                    tried_fallback = False

                    buf = b""
                    last_chunk_time = time.time()
                    while self._running:
                        # Periodically verify stream is still providing data
                        chunk = resp.read(16384)
                        if not chunk:
                            log.warning("Stream ended by server")
                            break
                        buf += chunk

                        # Find JPEG start and end
                        soi = buf.find(b"\xff\xd8")
                        eoi = buf.find(b"\xff\xd9")
                        if soi == -1 or eoi == -1 or eoi <= soi:
                            # Bound memory growth
                            if len(buf) > 1_000_000:
                                buf = buf[-500_000:]
                            continue

                        jpg_bytes = buf[soi:eoi + 2]
                        buf = buf[eoi + 2:]

                        frame = cv2.imdecode(
                            np.frombuffer(jpg_bytes, dtype=np.uint8),
                            cv2.IMREAD_COLOR
                        )
                        if frame is None:
                            continue

                        now = time.time()
                        fps_count += 1
                        elapsed = now - fps_t0
                        calc_fps = self._status.fps
                        if elapsed >= 1.0:
                            calc_fps = fps_count / elapsed
                            fps_count = 0
                            fps_t0 = now

                        iso_ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

                        with self._lock:
                            self._last_frame = frame
                            self._last_jpeg = jpg_bytes
                            self._status.frames_received += 1
                            self._status.last_frame_ts = now
                            self._status.last_frame_iso = iso_ts
                            self._status.fps = calc_fps

            except urllib.error.HTTPError as he:
                err_msg = f"HTTP {he.code}: {he.reason}"
                log.warning(f"Pi stream error: {err_msg}")
                # If /video returns 404, attempt fallback to ROS 2 endpoint
                if he.code == 404 and not tried_fallback and not self.url_override:
                    active_url = self.get_fallback_stream_url()
                    tried_fallback = True
                    log.info(f"Falling back to ROS stream endpoint: {active_url}")
                with self._lock:
                    self._status.connected = False
                    self._status.error_message = err_msg
                    self._status.reconnect_count += 1
            except Exception as e:
                err_msg = str(e)
                log.warning(f"Pi stream disconnect: {err_msg}")
                with self._lock:
                    self._status.connected = False
                    self._status.error_message = err_msg
                    self._status.reconnect_count += 1

            if self._running:
                time.sleep(backoff)
                backoff = min(backoff * 1.5, 4.0)

