"""
evaluation.py

Evaluation metrics calculation and plot generation suite.
Produces quantitative metrics (RMSE, detection rate, success rate) and multi-panel plots.
"""

import os
from typing import Dict, Any
import numpy as np
import matplotlib.pyplot as plt

class SystemEvaluator:
    """Evaluates closed-loop tracking metrics and generates visualization plots."""

    def __init__(self, output_dir: str = "output"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def compute_metrics(self, data_dict: Dict[str, np.ndarray]) -> Dict[str, Any]:
        """Calculates comprehensive quantitative metrics from simulation data dictionary."""
        if not data_dict or "time" not in data_dict:
            return {}

        t = data_dict["time"]
        dt = t[1] - t[0] if len(t) > 1 else 0.002

        uav_pos = np.column_stack((data_dict["uav_x"], data_dict["uav_y"], data_dict["uav_z"]))
        tgt_pos = np.column_stack((data_dict["target_x"], data_dict["target_y"], data_dict["target_z"]))

        # 3D Position Error
        pos_errors = np.linalg.norm(uav_pos - tgt_pos, axis=1)
        mean_pos_error = float(np.mean(pos_errors))
        rmse_pos_error = float(np.sqrt(np.mean(pos_errors ** 2)))
        max_pos_error = float(np.max(pos_errors))

        # Target detection & tracking success rate
        detected = data_dict["target_detected"] == 1
        detection_rate = float(np.mean(detected) * 100.0)

        dist_errors = np.abs(data_dict["distance_error"])
        mean_dist_error = float(np.mean(dist_errors[detected])) if np.any(detected) else 0.0

        pixel_errors_x = data_dict["pixel_error_x"]
        pixel_errors_y = data_dict["pixel_error_y"]
        pixel_errors = np.sqrt(pixel_errors_x**2 + pixel_errors_y**2)
        mean_pixel_error = float(np.mean(pixel_errors[detected])) if np.any(detected) else 0.0

        # Success requires both range and image-centering errors to be small.
        success_mask = detected & (dist_errors <= 0.5) & (pixel_errors <= 20.0)
        tracking_success_rate = float(np.mean(success_mask) * 100.0)

        # Target loss transitions
        loss_transitions = np.where((detected[:-1] == 1) & (detected[1:] == 0))[0]
        reacquire_transitions = np.where((detected[:-1] == 0) & (detected[1:] == 1))[0]
        loss_count = len(loss_transitions)

        # Average reacquisition time
        reacquire_times = []
        for loss_idx in loss_transitions:
            reacq_candidates = reacquire_transitions[reacquire_transitions > loss_idx]
            if len(reacq_candidates) > 0:
                reacq_idx = reacq_candidates[0]
                reacquire_times.append((reacq_idx - loss_idx) * dt)
        avg_reacquire_time = float(np.mean(reacquire_times)) if reacquire_times else 0.0

        avg_uav_target_dist = float(np.mean(pos_errors))

        metrics = {
            "detection_rate_pct": detection_rate,
            "tracking_success_rate_pct": tracking_success_rate,
            "mean_pos_error_m": mean_pos_error,
            "rmse_pos_error_m": rmse_pos_error,
            "max_pos_error_m": max_pos_error,
            "mean_dist_error_m": mean_dist_error,
            "mean_pixel_error_px": mean_pixel_error,
            "target_loss_count": loss_count,
            "avg_reacquisition_time_sec": avg_reacquire_time,
            "avg_uav_target_dist_m": avg_uav_target_dist
        }
        return metrics

    def generate_plots(self, data_dict: Dict[str, np.ndarray], trajectory_name: str = "tracking") -> str:
        """Generates comprehensive summary evaluation plot and saves image."""
        if not data_dict or "time" not in data_dict:
            return ""

        t = data_dict["time"]
        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle(f"MuJoCo UAV Closed-Loop Target Tracking Evaluation ({trajectory_name.upper()})", fontsize=14, fontweight='bold')

        # 1. 2D XY Trajectories
        ax = axes[0, 0]
        ax.plot(data_dict["target_x"], data_dict["target_y"], 'r--', label='Target Path', linewidth=2)
        ax.plot(data_dict["uav_x"], data_dict["uav_y"], 'b-', label='UAV Path', linewidth=1.5)
        ax.set_title("2D Trajectory (XY Plane)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend()
        ax.grid(True)

        # 2. 3D Altitude Profile
        ax = axes[0, 1]
        ax.plot(t, data_dict["target_z"], 'r--', label='Target Z')
        ax.plot(t, data_dict["uav_z"], 'b-', label='UAV Z')
        ax.set_title("Altitude (Z Profile)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude (m)")
        ax.legend()
        ax.grid(True)

        # 3. Position Error over time
        ax = axes[1, 0]
        uav_pos = np.column_stack((data_dict["uav_x"], data_dict["uav_y"], data_dict["uav_z"]))
        tgt_pos = np.column_stack((data_dict["target_x"], data_dict["target_y"], data_dict["target_z"]))
        pos_err = np.linalg.norm(uav_pos - tgt_pos, axis=1)
        ax.plot(t, pos_err, 'g-', label='3D Position Error')
        ax.set_title("3D Target-UAV Position Error (m)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.legend()
        ax.grid(True)

        # 4. Pixel Errors over time
        ax = axes[1, 1]
        ax.plot(t, data_dict["pixel_error_x"], 'c-', label='Pixel Error X')
        ax.plot(t, data_dict["pixel_error_y"], 'm-', label='Pixel Error Y')
        ax.set_title("Camera Pixel Errors (px)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pixel Error (px)")
        ax.legend()
        ax.grid(True)

        # 5. UAV Velocity Commands
        ax = axes[2, 0]
        ax.plot(t, data_dict["vx"], 'b-', label='Vx (Forward)')
        ax.plot(t, data_dict["vy"], 'g-', label='Vy (Lateral)')
        ax.plot(t, data_dict["vz"], 'r-', label='Vz (Vertical)')
        ax.set_title("UAV Command Velocities (m/s)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend()
        ax.grid(True)

        # 6. Yaw Error over time
        ax = axes[2, 1]
        ax.plot(t, np.degrees(data_dict["yaw_error"]), 'k-', label='Yaw Error (deg)')
        ax.set_title("Yaw Error (degrees)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Yaw Error (deg)")
        ax.legend()
        ax.grid(True)

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plot_path = os.path.join(self.output_dir, f"tracking_results_{trajectory_name}.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()
        return plot_path
