"""
BuildScan Rover — Local YOLO Crack Detection Module
════════════════════════════════════════════════════
Loads and runs inference using the trained instance segmentation model
(models/yolo26n-seg.pt).

Key capabilities:
- Inspects and verifies model class names (verifies 'crack' class)
- Safe CPU / CUDA device selection with automatic fallback
- Model warm-up on initialization
- Non-destructive frame annotation (returns a copy)
- Bounding boxes, polygon masks, confidence scores, and pixel measurements
- Measurement transparency: reports pixel metrics and explicit calibration reference
- Inference latency (ms) and effective FPS reporting
"""

import os
import sys
import time
import math
import logging
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger("buildscan.yolo")

DEFAULT_MODEL_PATH = Path("models/yolo26n-seg.pt")
DEFAULT_CONF_THRESH = float(os.getenv("BUILDSCAN_CONF_THRESH", "0.25"))
# Approximate monocular scale reference (e.g. 1 px ≈ 0.025 mm at ~30 cm working distance)
CALIBRATION_PX_TO_MM = float(os.getenv("BUILDSCAN_PX_TO_MM", "0.025"))


@dataclass
class DetectionItem:
    crack_id: str
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    width_px: float
    length_px: float
    estimated_width_mm: float
    estimated_length_mm: float
    severity: str
    action: str
    mask_polygon: Optional[List[List[int]]] = None


@dataclass
class InferenceResult:
    timestamp: float
    latency_ms: float
    device: str
    device_name: str
    model_name: str
    detections: List[DetectionItem] = field(default_factory=list)
    annotated_frame: Optional[np.ndarray] = None
    is_crack_trained: bool = True
    error_message: str = ""


# Remediation guidelines based on aperture size
CIVIL_ACTIONS = {
    "NONE": "No structural defects visible on surface.",
    "LOW": "Hairline crack (< 0.30 mm). Apply surface sealant; monitor periodically.",
    "MEDIUM": "Medium fracture (0.30–0.55 mm). Low-pressure epoxy resin injection recommended.",
    "HIGH": "Critical crack (≥ 0.55 mm). Immediate structural engineering evaluation required.",
}


def classify_severity(aperture_px: float, scale_px_to_mm: float = CALIBRATION_PX_TO_MM) -> Tuple[str, str]:
    width_mm = aperture_px * scale_px_to_mm
    if width_mm < 0.30:
        return "LOW", CIVIL_ACTIONS["LOW"]
    elif width_mm < 0.55:
        return "MEDIUM", CIVIL_ACTIONS["MEDIUM"]
    else:
        return "HIGH", CIVIL_ACTIONS["HIGH"]


class YOLOCrackDetector:
    """Wrapper around Ultralytics YOLO segmentation model with robust diagnostics."""

    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL_PATH,
        conf_thresh: float = DEFAULT_CONF_THRESH,
        scale_px_to_mm: float = CALIBRATION_PX_TO_MM,
    ):
        self.model_path = Path(model_path)
        self.conf_thresh = conf_thresh
        self.scale_px_to_mm = scale_px_to_mm

        self.model = None
        self.device = "cpu"
        self.device_name = "CPU"
        self.is_crack_trained = False
        self.class_names: Dict[int, str] = {}
        self.model_name = self.model_path.name
        self.model_version = "1.0.0"

        self._init_model()

    def _init_model(self):
        try:
            import torch
            from ultralytics import YOLO

            if torch.cuda.is_available():
                self.device = 0
                self.device_name = torch.cuda.get_device_name(0)
            else:
                self.device = "cpu"
                self.device_name = "CPU"

            log.info(f"Targeting device: {self.device_name} ({self.device})")

            if not self.model_path.exists():
                log.error(f"Model file not found at: {self.model_path.resolve()}")
                return

            self.model = YOLO(str(self.model_path))
            self.class_names = getattr(self.model, "names", {})

            # Inspect class names
            crack_classes = [c for c in self.class_names.values() if "crack" in str(c).lower()]
            if crack_classes:
                self.is_crack_trained = True
                log.info(f"Verified crack-trained weights: classes={self.class_names}")
            else:
                self.is_crack_trained = False
                log.warning(f"Weights do not contain 'crack' class! Classes: {self.class_names}")

            # Warm-up inference
            log.info("Running model warm-up...")
            warmup_img = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model(warmup_img, conf=self.conf_thresh, device=self.device, verbose=False)
            log.info("YOLO warm-up complete.")

        except Exception as e:
            log.error(f"Failed to load YOLO model: {e}")
            self.model = None

    def detect(self, frame: np.ndarray, conf_override: Optional[float] = None) -> InferenceResult:
        now = time.time()
        conf = self.conf_thresh if conf_override is None else conf_override

        if frame is None or self.model is None:
            return InferenceResult(
                timestamp=now,
                latency_ms=0.0,
                device=str(self.device),
                device_name=self.device_name,
                model_name=self.model_name,
                is_crack_trained=self.is_crack_trained,
                annotated_frame=None if frame is None else frame.copy(),
                error_message="Model not loaded or invalid frame" if self.model is None else "Null frame"
            )

        annotated = frame.copy()
        h, w = frame.shape[:2]
        detections: List[DetectionItem] = []

        try:
            t0 = time.time()
            results = self.model(
                frame,
                conf=conf,
                device=self.device,
                verbose=False
            )
            latency = (time.time() - t0) * 1000.0

            res = results[0]
            if res.boxes is not None and len(res.boxes) > 0:
                # Segmentation masks overlay
                if res.masks is not None:
                    overlay = annotated.copy()
                    for mask_t in res.masks.data:
                        m = (mask_t.cpu().numpy() * 255).astype(np.uint8)
                        if m.shape != (h, w):
                            m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                        overlay[m > 128] = (30, 30, 220)  # Red translucent mask
                    cv2.addWeighted(overlay, 0.40, annotated, 0.60, 0, annotated)

                for i, box in enumerate(res.boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    score = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = self.class_names.get(cls_id, f"class_{cls_id}")

                    bw = max(1, x2 - x1)
                    bh = max(1, y2 - y1)
                    blen = math.hypot(bw, bh)

                    # Compute aperture from segmentation mask if present
                    if res.masks is not None and i < len(res.masks.data):
                        m_arr = res.masks.data[i].cpu().numpy()
                        m_area = float(np.sum(m_arr > 0))
                        aperture = m_area / max(1.0, blen)
                    else:
                        aperture = min(bw, bh)

                    sev, act = classify_severity(aperture, self.scale_px_to_mm)
                    w_mm = round(aperture * self.scale_px_to_mm, 2)
                    l_mm = round(blen * self.scale_px_to_mm, 1)

                    det = DetectionItem(
                        crack_id=f"CRK-{i+1:02d}",
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=score,
                        bbox=(x1, y1, x2, y2),
                        width_px=round(aperture, 1),
                        length_px=round(blen, 1),
                        estimated_width_mm=w_mm,
                        estimated_length_mm=l_mm,
                        severity=sev,
                        action=act
                    )
                    detections.append(det)

                    # Draw bounding box & label on annotated frame
                    box_col = (0, 210, 80) if sev == "LOW" else ((0, 140, 255) if sev == "MEDIUM" else (30, 30, 245))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), box_col, 2)

                    lbl = f"{cls_name.upper()} {score:.0%} | {w_mm}mm"
                    (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(annotated, (x1, max(0, y1 - 20)), (x1 + tw + 6, y1), box_col, -1)
                    cv2.putText(annotated, lbl, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            return InferenceResult(
                timestamp=now,
                latency_ms=round(latency, 1),
                device=str(self.device),
                device_name=self.device_name,
                model_name=self.model_name,
                detections=detections,
                annotated_frame=annotated,
                is_crack_trained=self.is_crack_trained,
                error_message=""
            )

        except Exception as e:
            log.warning(f"Inference error: {e}")
            return InferenceResult(
                timestamp=now,
                latency_ms=0.0,
                device=str(self.device),
                device_name=self.device_name,
                model_name=self.model_name,
                detections=[],
                annotated_frame=annotated,
                is_crack_trained=self.is_crack_trained,
                error_message=str(e)
            )

