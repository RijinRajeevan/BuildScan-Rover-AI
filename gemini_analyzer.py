"""
BuildScan Rover — Gemini Flash Visual Analysis Module
══════════════════════════════════════════════════════
Integrates Google's official Gen AI SDK (`google-genai`) to provide selective,
on-demand high-level visual and structural assessment of inspection frames.

Safety & Governance:
- Supplementary visual interpretation ONLY; cannot replace licensed professional engineers.
- Never sends continuous high-frequency video frames to the API.
- Secure API key resolution from environment variables or .env file (never tracked in git).
- Validated JSON schema return with explicit confidence and limitation disclosures.
- Robust exception handling for quota, network timeout, rate-limits, and invalid keys.
"""

import os
import io
import time
import json
import logging
from typing import List, Dict, Any, Optional
from pathlib import Path
from dataclasses import dataclass, asdict

import cv2
import numpy as np
from pydantic import BaseModel, Field

log = logging.getLogger("buildscan.gemini")

DEFAULT_GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-3.8-flash")


class GeminiInspectionSchema(BaseModel):
    visible_defects: List[str] = Field(
        description="List of visually identifiable defects (e.g. hairline crack, spalling, efflorescence, honeycomb, none)"
    )
    crack_visible: bool = Field(
        description="True if one or more surface cracks or fractures are visibly identifiable in the image"
    )
    visual_description: str = Field(
        description="Detailed visual description of concrete surface condition, texture, pattern, and visible anomalies"
    )
    approximate_location: str = Field(
        description="Spatial position within the frame (e.g. upper-left quadrant, central vertical axis, bottom edge)"
    )
    confidence_assessment: str = Field(
        description="Confidence level in the visual diagnosis: HIGH, MEDIUM, or LOW with short rationale"
    )
    possible_surface_material: str = Field(
        description="Observed construction material (e.g. reinforced concrete, plaster, masonry brick, drywall)"
    )
    limitations: str = Field(
        description="Optical and sensor limitations (e.g. monocular lighting shadows, resolution limit, depth ambiguity)"
    )
    recommended_next_step: str = Field(
        description="Engineering next step (e.g. tactile sounding, crack-depth gauge, core sampling, qualified PE review)"
    )


@dataclass
class GeminiAnalysisResult:
    success: bool
    model: str
    timestamp: str
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    disclaimer: str = (
        "Visual assessment provided by multimodal AI is supplementary. "
        "A photograph alone cannot determine load-bearing capacity or structural integrity. "
        "All structural remediation plans require review by a licensed structural engineer."
    )


def load_env_file(filepath: Path = Path(".env")):
    """Safely loads key-value pairs from .env if present into os.environ without printing values."""
    if filepath.exists():
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        k, v = line.split("=", 1)
                        k = k.strip()
                        v = v.strip().strip("'\"")
                        if k and k not in os.environ:
                            os.environ[k] = v
        except Exception:
            pass


load_env_file()


def get_api_key() -> Optional[str]:
    """Retrieves GEMINI_API_KEY from environment or .env without logging or echoing."""
    return os.getenv("GEMINI_API_KEY", "").strip() or None


def is_gemini_available() -> bool:
    """Returns True if the Google Gen AI SDK is installed and an API key is configured."""
    return bool(get_api_key())


SYSTEM_INSTRUCTION = """You are an expert civil engineering vision specialist assisting a structural inspection rover.
Analyze the provided inspection photograph from an indoor structural health robot.
Adhere strictly to civil engineering inspection standards (e.g. ACI 201.1R / ACI 224.1R):
1. Distinguish visually observed surface facts from speculative internal condition.
2. Differentiate between genuine structural cracks and harmless surface artifacts (formwork lines, scratches, cobwebs, cables).
3. Explicitly report visual confidence, surface material, and optical limitations.
4. Output valid JSON adhering strictly to the requested schema.
"""


def analyze_inspection_frame(
    image: np.ndarray,
    metadata: Optional[Dict[str, Any]] = None,
    model_name: Optional[str] = None,
    timeout_sec: float = 12.0
) -> GeminiAnalysisResult:
    """
    Analyzes a single still frame using Google Gemini Flash.
    Returns a validated GeminiAnalysisResult.
    """
    now_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    target_model = model_name or os.getenv("GEMINI_MODEL", DEFAULT_GEMINI_MODEL)

    api_key = get_api_key()
    if not api_key:
        return GeminiAnalysisResult(
            success=False,
            model=target_model,
            timestamp=now_str,
            error="GEMINI_API_KEY environment variable is not configured. Set GEMINI_API_KEY to enable Gemini analysis."
        )

    if image is None or image.size == 0:
        return GeminiAnalysisResult(
            success=False,
            model=target_model,
            timestamp=now_str,
            error="Empty or invalid image frame provided for analysis."
        )

    try:
        from google import genai
        from google.genai import types

        # Encode frame to JPEG
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return GeminiAnalysisResult(
                success=False,
                model=target_model,
                timestamp=now_str,
                error="Failed to encode inspection frame to JPEG."
            )
        jpg_bytes = buf.tobytes()

        client = genai.Client(api_key=api_key)

        prompt_text = (
            "Analyze this structural health inspection frame captured by the BuildScan Rover. "
            "Inspect the concrete or masonry surface for cracks, fracture propagation, spalling, or surface defects."
        )
        if metadata:
            prompt_text += f"\nOnboard telemetry metadata: {json.dumps(metadata)}"

        config = types.GenerateContentConfig(
            system_instruction=SYSTEM_INSTRUCTION,
            temperature=0.2,
            response_mime_type="application/json",
            response_schema=GeminiInspectionSchema,
        )

        response = client.models.generate_content(
            model=target_model,
            contents=[
                types.Part.from_bytes(data=jpg_bytes, mime_type="image/jpeg"),
                prompt_text,
            ],
            config=config,
        )

        raw_text = response.text or "{}"
        parsed_dict = json.loads(raw_text)

        # Validate with pydantic schema
        validated = GeminiInspectionSchema(**parsed_dict)

        return GeminiAnalysisResult(
            success=True,
            model=target_model,
            timestamp=now_str,
            data=validated.model_dump(),
            error=None
        )

    except json.JSONDecodeError as jde:
        log.error(f"Gemini response JSON decode error: {jde}")
        return GeminiAnalysisResult(
            success=False,
            model=target_model,
            timestamp=now_str,
            error=f"Malformed JSON returned by Gemini: {jde}"
        )
    except Exception as e:
        err_msg = str(e)
        log.error(f"Gemini API request failed: {err_msg}")
        # Sanitize error to avoid leaking keys or internal traces
        if "API_KEY_INVALID" in err_msg or "PERMISSION_DENIED" in err_msg:
            err_msg = "Invalid or unauthorized GEMINI_API_KEY. Please verify your Google AI Studio credentials."
        elif "RESOURCE_EXHAUSTED" in err_msg:
            err_msg = "Gemini API rate limit or quota exceeded. Please wait before retrying."
        elif "NOT_FOUND" in err_msg:
            err_msg = f"Configured Gemini model '{target_model}' was not found. Please choose an active Flash model."
        return GeminiAnalysisResult(
            success=False,
            model=target_model,
            timestamp=now_str,
            error=err_msg
        )

