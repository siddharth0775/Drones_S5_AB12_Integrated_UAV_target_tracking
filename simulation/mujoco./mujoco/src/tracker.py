"""
tracker.py

Target tracker module implementing state machine (TRACKING, SEARCHING, LOST),
exponential temporal smoothing, and lost-target timeout handling.
"""

from enum import Enum
import numpy as np
from src.detector import DetectionResult

class TrackingState(Enum):
    TRACKING = "TRACKING"
    SEARCHING = "SEARCHING"
    LOST = "LOST"

class TargetTracker:
    """Manages continuous temporal tracking and state transition machine."""

    def __init__(self, smoothing: float = 0.35, lost_timeout_sec: float = 2.0, dt: float = 0.002):
        self.smoothing = smoothing
        self.lost_timeout_sec = lost_timeout_sec
        self.dt = dt

        self.state = TrackingState.SEARCHING
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None

        self.lost_frames = 0
        self.tracking_frames = 0
        self.last_known_dir = 1.0  # 1.0 for right, -1.0 for left search direction

    def reset(self):
        """Resets tracker state."""
        self.state = TrackingState.SEARCHING
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None
        self.lost_frames = 0
        self.tracking_frames = 0

    def update(self, detection: DetectionResult, frame_w: int, frame_h: int) -> dict:
        """
        Updates tracker state with new detection result.
        Returns dictionary containing state, smoothed pixel coordinates, radius, and confidence.
        """
        if not detection.found:
            self.lost_frames += 1
            self.tracking_frames = 0
            
            lost_time = self.lost_frames * self.dt
            if lost_time < self.lost_timeout_sec:
                self.state = TrackingState.SEARCHING
            else:
                self.state = TrackingState.LOST
                # Reset filtered coordinates when lost completely
                self.filtered_cx = None
                self.filtered_cy = None
                self.filtered_radius = None

            return {
                "state": self.state.value,
                "found": False,
                "cx": self.filtered_cx,
                "cy": self.filtered_cy,
                "radius_px": self.filtered_radius,
                "lost_frames": self.lost_frames,
                "search_dir": self.last_known_dir
            }

        # Target is detected
        self.state = TrackingState.TRACKING
        self.lost_frames = 0
        self.tracking_frames += 1

        cx, cy, radius = detection.cx, detection.cy, detection.radius_px

        # Temporal low-pass smoothing
        if self.filtered_cx is None:
            self.filtered_cx = cx
            self.filtered_cy = cy
            self.filtered_radius = radius
        else:
            a = self.smoothing
            self.filtered_cx = a * cx + (1.0 - a) * self.filtered_cx
            self.filtered_cy = a * cy + (1.0 - a) * self.filtered_cy
            self.filtered_radius = a * radius + (1.0 - a) * self.filtered_radius

        # Record direction relative to center for search heuristic
        center_x = frame_w / 2.0
        if abs(self.filtered_cx - center_x) > 10.0:
            self.last_known_dir = -1.0 if (self.filtered_cx > center_x) else 1.0

        return {
            "state": self.state.value,
            "found": True,
            "cx": float(self.filtered_cx),
            "cy": float(self.filtered_cy),
            "radius_px": float(self.filtered_radius),
            "lost_frames": 0,
            "confidence": detection.confidence
        }
