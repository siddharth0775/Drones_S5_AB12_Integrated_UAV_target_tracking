"""
detector.py

Target detection module using HSV color thresholding, contour extraction,
bounding box and center calculation.
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import cv2
import numpy as np

@dataclass
class DetectionResult:
    found: bool
    cx: Optional[float] = None
    cy: Optional[float] = None
    radius_px: Optional[float] = None
    bbox: Optional[Tuple[int, int, int, int]] = None # (xmin, ymin, xmax, ymax)
    confidence: float = 0.0
    area: float = 0.0

class ColorBlobDetector:
    """Detects the magenta box target in RGB camera images using HSV segmentation."""

    def __init__(
        self,
        lower_hsv=(135, 80, 80),
        upper_hsv=(170, 255, 255),
        min_area=15.0,
    ):
        self.lower = np.array(lower_hsv, dtype=np.uint8)
        self.upper = np.array(upper_hsv, dtype=np.uint8)
        self.min_area = min_area

    def detect(self, rgb_frame: np.ndarray) -> DetectionResult:
        """
        Processes RGB image frame and returns target detection result.
        """
        if rgb_frame is None or rgb_frame.size == 0:
            return DetectionResult(found=False)

        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # Morphological noise filtering
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return DetectionResult(found=False)

        # Filter contours by minimum area
        candidates = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not candidates:
            return DetectionResult(found=False)

        # Find largest contour (most prominent target candidate)
        target_contour = max(candidates, key=cv2.contourArea)
        area = cv2.contourArea(target_contour)

        (cx, cy), radius = cv2.minEnclosingCircle(target_contour)
        if radius <= 1.0:
            return DetectionResult(found=False)

        x, y, w, h = cv2.boundingRect(target_contour)
        bbox = (int(x), int(y), int(x + w), int(y + h))

        # Calculate confidence score based on circularity and area
        perimeter = cv2.arcLength(target_contour, True)
        circularity = (4.0 * np.pi * area) / (perimeter * perimeter + 1e-6)
        confidence = float(np.clip(circularity, 0.0, 1.0))

        return DetectionResult(
            found=True,
            cx=float(cx),
            cy=float(cy),
            radius_px=float(radius),
            bbox=bbox,
            confidence=confidence,
            area=float(area)
        )
