"""
logger.py

Structured data logging system for MuJoCo UAV Target Tracking simulation.
Records high-frequency telemetry at every timestep and exports to CSV and dict data structures.
"""

import csv
from typing import List, Dict, Any
import numpy as np

class DataLogger:
    """Logs detailed step-by-step simulation variables."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Clears logged records."""
        self.records: List[Dict[str, Any]] = []

    def log_step(
        self,
        t: float,
        target_pos: np.ndarray,
        uav_pos: np.ndarray,
        rel_pos: np.ndarray,
        pixel_error_x: float,
        pixel_error_y: float,
        distance_error: float,
        yaw_error: float,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
        target_detected: bool,
        tracking_confidence: float,
        tracking_state: str = "TRACKING"
    ):
        """Appends a new timestep telemetry frame."""
        record = {
            "time": float(t),
            "target_x": float(target_pos[0]),
            "target_y": float(target_pos[1]),
            "target_z": float(target_pos[2]),
            "uav_x": float(uav_pos[0]),
            "uav_y": float(uav_pos[1]),
            "uav_z": float(uav_pos[2]),
            "relative_x": float(rel_pos[0]) if rel_pos is not None else 0.0,
            "relative_y": float(rel_pos[1]) if rel_pos is not None else 0.0,
            "relative_z": float(rel_pos[2]) if rel_pos is not None else 0.0,
            "pixel_error_x": float(pixel_error_x),
            "pixel_error_y": float(pixel_error_y),
            "distance_error": float(distance_error),
            "yaw_error": float(yaw_error),
            "vx": float(vx),
            "vy": float(vy),
            "vz": float(vz),
            "yaw_rate": float(yaw_rate),
            "target_detected": int(target_detected),
            "tracking_confidence": float(tracking_confidence),
            "tracking_state": str(tracking_state)
        }
        self.records.append(record)

    def save_to_csv(self, filepath: str):
        """Saves recorded history to a CSV file."""
        if not self.records:
            return
        keys = self.records[0].keys()
        with open(filepath, "w", newline="") as f:
            dict_writer = csv.DictWriter(f, fieldnames=keys)
            dict_writer.writeheader()
            dict_writer.writerows(self.records)

    def get_as_dict(self) -> Dict[str, np.ndarray]:
        """Converts history list to dictionary of NumPy 1D arrays."""
        if not self.records:
            return {}
        keys = self.records[0].keys()
        return {
            k: np.array([r[k] for r in self.records])
            for k in keys if k != "tracking_state"
        }
