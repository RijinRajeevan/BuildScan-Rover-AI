#!/usr/bin/env python3
"""
report_generator.py  ── Stage 7
════════════════════════════════
BuildScan Rover – PDF Report Generator

Refactored from original app.py generate_report() function.
Now a standalone class called by the inspection action server.

Preserves the original ReportLab PDF layout and extends it with:
  - Inspection ID
  - Area name
  - Original image + annotated image
  - Model information
  - Structured sections

Usage (called by inspection_manager.py):
  from buildscan_inspection.report_generator import ReportGenerator
  gen = ReportGenerator(output_dir='/tmp/buildscan_reports')
  pdf_path = gen.generate(inspection_data)
"""

import os
import cv2
import uuid
import datetime
import tempfile
from dataclasses import dataclass, field
from typing import Optional

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas
    from reportlab.lib.colors import HexColor, white, black
    from reportlab.lib.utils import ImageReader
    from reportlab.lib.units import mm
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False


@dataclass
class InspectionData:
    """Data container for a completed inspection."""
    area_name:           str
    inspection_id:       str = field(default_factory=lambda: str(uuid.uuid4())[:8].upper())
    timestamp:           datetime.datetime = field(default_factory=datetime.datetime.now)

    # Detection results
    cracks_detected:     int   = 0
    average_confidence:  float = 0.0    # 0-100%
    maximum_width:       float = 0.0    # pixels
    total_length:        float = 0.0    # pixels
    severity:            str   = 'NONE'
    recommendation:      str   = 'No crack detected'

    # Images (OpenCV BGR numpy arrays or file paths)
    original_image:      Optional[object] = None
    annotated_image:     Optional[object] = None
    original_image_path: str = ''
    annotated_image_path: str = ''

    # Model info
    model_name:          str = 'YOLO26n-seg'
    model_version:       str = '1.0.0'
    confidence_threshold: float = 0.30


class ReportGenerator:
    """
    Generates PDF inspection reports using ReportLab.

    Preserves original BuildScan visual style (blue header, colored cards)
    and extends with full inspection metadata.
    """

    # Color palette (from original app.py)
    COLOR_HEADER  = HexColor('#0ea5e9')   # sky blue
    COLOR_ACCENT  = HexColor('#1e3a8a')   # dark blue
    COLOR_BG      = HexColor('#f4f4f4')   # light grey
    COLOR_DARK    = HexColor('#111827')   # near black
    COLOR_BLUE    = HexColor('#2563eb')
    COLOR_GREEN   = HexColor('#16a34a')
    COLOR_ORANGE  = HexColor('#ea580c')
    COLOR_PURPLE  = HexColor('#7c3aed')
    COLOR_RED     = HexColor('#dc2626')

    SEVERITY_COLORS = {
        'NONE':   HexColor('#6b7280'),
        'LOW':    HexColor('#16a34a'),
        'MEDIUM': HexColor('#ea580c'),
        'HIGH':   HexColor('#dc2626'),
    }

    def __init__(self, output_dir: str = '/tmp/buildscan_reports'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate(self, data: InspectionData) -> str:
        """
        Generate a PDF report for the inspection.

        Returns:
            str: Absolute path to the generated PDF file.
        """
        if not REPORTLAB_AVAILABLE:
            raise RuntimeError(
                'reportlab not installed. Run: pip install reportlab')

        # ── Save images to temp files if provided as arrays ───────────────────
        orig_path = self._save_image(
            data.original_image, data.original_image_path, 'original')
        ann_path  = self._save_image(
            data.annotated_image, data.annotated_image_path, 'annotated')

        # ── Output filename ───────────────────────────────────────────────────
        ts = data.timestamp.strftime('%Y%m%d_%H%M%S')
        filename = f'BuildScan_{data.inspection_id}_{data.area_name}_{ts}.pdf'
        filepath = os.path.join(self.output_dir, filename)

        # ── Build PDF ─────────────────────────────────────────────────────────
        c = canvas.Canvas(filepath, pagesize=A4)
        w, h = A4

        self._draw_page(c, w, h, data, orig_path, ann_path)
        c.save()

        return filepath

    # ──────────────────────────────────────────────────────────────────────────
    # DRAWING
    # ──────────────────────────────────────────────────────────────────────────

    def _draw_page(self, c, w, h, data: InspectionData,
                   orig_path: str, ann_path: str):
        """Draw the complete inspection report page."""

        # ── Background ────────────────────────────────────────────────────────
        c.setFillColor(self.COLOR_BG)
        c.rect(0, 0, w, h, fill=1, stroke=0)

        # ── Header bar ────────────────────────────────────────────────────────
        c.setFillColor(self.COLOR_ACCENT)
        c.rect(0, h - 80, w, 80, fill=1, stroke=0)

        c.setFillColor(white)
        c.setFont('Helvetica-Bold', 26)
        c.drawString(30, h - 45, '🏗 BuildScan Rover')

        c.setFont('Helvetica', 13)
        c.drawString(30, h - 68, 'AI-Based Structural Health Inspection Report')

        # Inspection ID top right
        c.setFont('Helvetica-Bold', 10)
        c.drawRightString(w - 20, h - 35, f'ID: {data.inspection_id}')
        c.setFont('Helvetica', 9)
        c.drawRightString(w - 20, h - 50, data.timestamp.strftime('%d-%m-%Y  %I:%M %p'))

        # ── Section: Area + Metadata ──────────────────────────────────────────
        y = h - 105
        c.setFillColor(self.COLOR_DARK)
        c.setFont('Helvetica-Bold', 11)
        c.drawString(30, y, f'Inspected Area:')
        c.setFont('Helvetica', 11)
        c.drawString(150, y, data.area_name)

        c.setFont('Helvetica-Bold', 11)
        c.drawString(30, y - 18, f'Model:')
        c.setFont('Helvetica', 11)
        c.drawString(150, y - 18, f'{data.model_name}  v{data.model_version}')

        c.setFont('Helvetica-Bold', 11)
        c.drawString(30, y - 36, 'Confidence Threshold:')
        c.setFont('Helvetica', 11)
        c.drawString(195, y - 36, f'{data.confidence_threshold:.2f}')

        # ── Divider ───────────────────────────────────────────────────────────
        c.setStrokeColor(self.COLOR_ACCENT)
        c.line(30, y - 50, w - 30, y - 50)

        # ── Severity Badge ────────────────────────────────────────────────────
        sev_color = self.SEVERITY_COLORS.get(data.severity, self.COLOR_RED)
        bx, by, bw, bh = w - 160, y - 90, 130, 35
        c.setFillColor(sev_color)
        c.roundRect(bx, by, bw, bh, 8, fill=1, stroke=0)
        c.setFillColor(white)
        c.setFont('Helvetica-Bold', 16)
        c.drawCentredString(bx + bw / 2, by + 10, f'⚠ {data.severity}')

        # ── Metric Cards ──────────────────────────────────────────────────────
        card_y = y - 65
        self._card(c, 30,  card_y, self.COLOR_BLUE,   'Cracks Detected',
                   str(data.cracks_detected))
        self._card(c, 175, card_y, self.COLOR_GREEN,  'Confidence',
                   f'{data.average_confidence:.1f}%')
        self._card(c, 320, card_y, self.COLOR_ORANGE, 'Max Width',
                   f'{data.maximum_width:.0f} px')

        card_y2 = card_y - 55
        self._card(c, 30,  card_y2, self.COLOR_PURPLE, 'Total Length',
                   f'{data.total_length:.0f} px')
        self._card(c, 175, card_y2, self.COLOR_RED,    'Recommendation',
                   data.recommendation[:28] + '…'
                   if len(data.recommendation) > 28 else data.recommendation)

        # ── Images ────────────────────────────────────────────────────────────
        img_y = card_y2 - 185
        img_h = 165
        img_w = (w - 80) / 2

        c.setFillColor(self.COLOR_DARK)
        c.setFont('Helvetica-Bold', 10)
        c.drawString(30, img_y + img_h + 5, 'Original Image')
        c.drawString(30 + img_w + 20, img_y + img_h + 5, 'Detected Cracks')

        if orig_path and os.path.exists(orig_path):
            c.drawImage(ImageReader(orig_path),
                        30, img_y, width=img_w, height=img_h,
                        preserveAspectRatio=True)

        if ann_path and os.path.exists(ann_path):
            c.drawImage(ImageReader(ann_path),
                        30 + img_w + 20, img_y, width=img_w, height=img_h,
                        preserveAspectRatio=True)

        # ── AI Performance Section ────────────────────────────────────────────
        perf_y = img_y - 60
        c.setFillColor(self.COLOR_DARK)
        c.setFont('Helvetica-Bold', 13)
        c.drawString(30, perf_y, 'AI Performance Metrics')

        c.setFont('Helvetica', 11)
        conf = data.average_confidence
        c.drawString(30,  perf_y - 20, f'Detection Confidence  :  {conf:.2f}%')
        c.drawString(30,  perf_y - 36, f'Measurements          :  Pixel-based (not physically calibrated)')
        c.drawString(30,  perf_y - 52, f'Max Crack Width       :  {data.maximum_width:.0f} px  |  Total Length: {data.total_length:.0f} px')

        # ── Footer ────────────────────────────────────────────────────────────
        c.setStrokeColor(self.COLOR_ACCENT)
        c.line(30, 50, w - 30, 50)
        c.setFillColor(self.COLOR_ACCENT)
        c.setFont('Helvetica-Oblique', 9)
        c.drawString(30, 35,
            'Generated by BuildScan Rover – AI Structural Health Inspection System')
        c.drawRightString(w - 30, 35,
            f'Inspection ID: {data.inspection_id}')

    def _card(self, c, x, y, color, title, value, cw=130, ch=45):
        """Draw a colored metric card (from original app.py style)."""
        c.setFillColor(color)
        c.roundRect(x, y, cw, ch, 8, fill=1, stroke=0)
        c.setFillColor(white)
        c.setFont('Helvetica-Bold', 10)
        c.drawString(x + 8, y + 28, title)
        c.setFont('Helvetica', 10)
        c.drawString(x + 8, y + 10, str(value))

    # ──────────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────────

    def _save_image(self, image_array, image_path: str, label: str) -> str:
        """Save a numpy image array to a temp file; or return existing path."""
        if image_path and os.path.exists(image_path):
            return image_path

        if image_array is not None:
            try:
                tmp = tempfile.NamedTemporaryFile(
                    delete=False, suffix='.jpg', prefix=f'buildscan_{label}_')
                cv2.imwrite(tmp.name, image_array)
                return tmp.name
            except Exception:
                pass

        return ''
