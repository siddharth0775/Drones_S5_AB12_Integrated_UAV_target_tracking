"""
simulation.py

Integrated Closed-Loop UAV Target Tracking Simulation Engine for MuJoCo.
Connects Target Trajectory -> MuJoCo Physics -> Live UAV Camera -> Vision Detector
-> Temporal Tracker -> 3D Localization -> Visual Servo Controller -> Quadrotor Actuators
-> Data Logger.

Feature parity with PyBullet track_demo.py
------------------------------------------
* OpenCV live camera window with identical HUD layout (DroneHUD)
* Keyboard-controlled manual target mode: W/S/A/D/R/F (matching PyBullet)
* 10 trajectory types (circle, figure8, helix, lissajous, zigzag, wander, ...)
* CSV telemetry logging (drone pos, target pos, errors, commands)
* Evaluation metrics + multi-panel matplotlib plots
"""

import time
import os
import mujoco
import mujoco.viewer
import numpy as np

from configs.config import SimConfig
from src.trajectory import get_trajectory, BaseTrajectory
from src.target import TargetManager
from src.camera import CameraManager
from src.detector import ColorBlobDetector
from src.tracker import TargetTracker
from src.coordinate_transform import CoordinateTransformer
from src.controller import ClosedLoopUAVController
from src.logger import DataLogger
from src.evaluation import SystemEvaluator
from src.hud import DroneHUD, parse_keyboard_target_delta


class UAVTrackingSimulation:
    """End-to-End MuJoCo Closed-Loop Visual Servo Simulation."""

    def __init__(
        self,
        trajectory_type: str = "circle",
        trajectory_kwargs: dict = None,
        config=SimConfig,
        model_path: str = None
    ):
        self.cfg = config
        self.trajectory_name = trajectory_type

        # Load MuJoCo model
        xml_path = model_path if model_path else config.MODEL_PATH
        if not os.path.exists(xml_path):
            if os.path.exists(os.path.join("models", "uav_target.xml")):
                xml_path = os.path.join("models", "uav_target.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = config.TIMESTEP
        self.data = mujoco.MjData(self.model)

        # Body & Sensor IDs
        self.uav_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        self.target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, config.TARGET_NAME)
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.CAMERA_NAME)
        self.disturbance_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("disturbance_panel", "disturbance_cylinder")
        ]
        self.rotor_sites = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"thrust_{i}")
            for i in range(1, 5)
        ]
        if any(geom_id < 0 for geom_id in self.disturbance_geom_ids):
            raise ValueError("The MuJoCo model must include both visual disturbance geometries.")

        # Pipeline modules
        traj_args = trajectory_kwargs if trajectory_kwargs is not None else {}
        self.trajectory = get_trajectory(trajectory_type, **traj_args)
        self.target_mgr = TargetManager(self.model, config.TARGET_NAME, self.trajectory)
        self.camera_mgr = CameraManager(self.model, config.CAMERA_NAME, config.RENDER_WIDTH, config.RENDER_HEIGHT)
        self.detector = ColorBlobDetector()
        self.tracker = TargetTracker(dt=config.TIMESTEP, lost_timeout_sec=config.TARGET_LOST_TIMEOUT)
        self.transformer = CoordinateTransformer(target_radius_real=config.TARGET_RADIUS_REAL)
        self.controller = ClosedLoopUAVController(config=config)
        self.logger = DataLogger()
        self.evaluator = SystemEvaluator()

        # HUD (matches PyBullet OpenCV window)
        self.hud = DroneHUD(width=config.RENDER_WIDTH, height=config.RENDER_HEIGHT)

        # Keyboard-controlled manual target position (mirrors PyBullet manual_target_pos)
        self._manual_target_pos = None

    def run(
        self,
        duration: float = 20.0,
        headless: bool = False,
        print_freq: float = 1.0,
        show_hud: bool = True,
        keyboard_mode: bool = False,
        target_move_speed: float = 1.5,
    ) -> dict:
        """
        Runs continuous closed-loop tracking simulation for specified duration (seconds).

        Parameters
        ----------
        duration        : Simulation duration in seconds.
        headless        : If True, runs without the MuJoCo interactive viewer window.
        print_freq      : Console status print interval in seconds.
        show_hud        : If True, shows OpenCV drone-camera HUD window (mirrors PyBullet).
        keyboard_mode   : If True, enables WASD/RF keyboard control of target position
                          (mirrors PyBullet track_demo.py).
        target_move_speed : Manual target movement speed in m/s (keyboard mode only).

        Returns
        -------
        dict : Evaluation metrics dictionary.
        """
        # Reset state
        mujoco.mj_resetData(self.model, self.data)

        # Initial UAV pose: hover at [0, 0, 2]
        self.data.qpos[0:3] = np.array([0.0, 0.0, 2.0])
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])  # Unit quaternion
        self.data.qvel[:] = 0.0

        self.logger.reset()
        self.tracker.reset()
        self.controller.reset()

        # Keyboard mode: start manual target at trajectory's t=0 position
        if keyboard_mode:
            self._manual_target_pos = self.trajectory.get_position(0.0).copy()
        else:
            self._manual_target_pos = None

        dt = self.cfg.TIMESTEP
        steps = int(duration / dt)
        last_print_time = 0.0

        print()
        print("=" * 65)
        print(f"  MUJOCO UAV TRACKING  |  Trajectory: {self.trajectory_name.upper()}")
        if keyboard_mode:
            print("  KEYBOARD MODE: W/S=fwd/bwd  A/D=left/right  R/F=up/dn  ESC=quit")
        print("=" * 65)

        if headless:
            for _step in range(steps):
                ctrl_out, tracker_info = self._simulation_step(
                    dt, keyboard_mode, target_move_speed, -1
                )
                if self.data.time - last_print_time >= print_freq:
                    self._print_status(tracker_info, ctrl_out)
                    last_print_time = self.data.time
        else:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and self.data.time < duration:
                    # Get keyboard input from HUD window if active
                    key = -1
                    if show_hud and keyboard_mode:
                        # key is read inside _simulation_step via HUD render
                        pass

                    ctrl_out, tracker_info = self._simulation_step(
                        dt, keyboard_mode, target_move_speed, key
                    )

                    if self.data.time - last_print_time >= print_freq:
                        self._print_status(tracker_info, ctrl_out)
                        last_print_time = self.data.time

                    viewer.sync()

                    # ESC from HUD closes simulation
                    if hasattr(self, '_last_key') and self._last_key == 27:
                        print("ESC pressed – stopping simulation.")
                        break

        self.camera_mgr.close()
        self.hud.close()

        # Compute metrics and generate summary plot
        data_dict = self.logger.get_as_dict()
        metrics = self.evaluator.compute_metrics(data_dict)
        plot_path = self.evaluator.generate_plots(data_dict, self.trajectory_name)

        # Save CSV output
        os.makedirs("output", exist_ok=True)
        csv_path = os.path.join("output", f"telemetry_{self.trajectory_name}.csv")
        self.logger.save_to_csv(csv_path)

        metrics["plot_path"] = plot_path
        metrics["csv_path"] = csv_path
        return metrics

    def _simulation_step(
        self,
        dt: float,
        keyboard_mode: bool,
        target_move_speed: float,
        _key_hint: int,
    ):
        """Single closed-loop simulation step."""

        # ---- 1. Update Target Position ----
        if keyboard_mode and self._manual_target_pos is not None:
            # Set target body position directly (mirrors PyBullet move_target_sphere)
            self.model.body_pos[self.target_body_id] = self._manual_target_pos
            mujoco.mj_forward(self.model, self.data)
            target_pos_world = self._manual_target_pos.copy()
        else:
            target_pos_world = self.target_mgr.update(self.data)

        t = self.data.time
        panel_id, cylinder_id = self.disturbance_geom_ids
        self.model.geom_pos[panel_id] = np.array([
            1.5,
            0.45 + 0.95 * np.sin(0.7 * t),
            2.5,
        ])
        self.model.geom_pos[cylinder_id] = np.array([
            2.1,
            0.45 + 0.85 * np.sin(0.5 * t + np.pi),
            2.5,
        ])
        mujoco.mj_forward(self.model, self.data)

        # ---- 2. Render Live UAV Camera Image ----
        rgb_frame = self.camera_mgr.capture_frame(self.data)

        # ---- 3. Vision Detection ----
        detection = self.detector.detect(rgb_frame)

        # ---- 4. Target Tracker (State Machine & Temporal Smoothing) ----
        tracker_info = self.tracker.update(detection, self.cfg.RENDER_WIDTH, self.cfg.RENDER_HEIGHT)

        rel_pos_body = None
        target_pos_world_est = None
        depth_est = 0.0
        pixel_error_x = 0.0
        pixel_error_y = 0.0

        if tracker_info["found"]:
            cx, cy = tracker_info["cx"], tracker_info["cy"]
            radius_px = tracker_info["radius_px"]

            # Pixel center errors
            center_x = self.cfg.RENDER_WIDTH / 2.0
            center_y = self.cfg.RENDER_HEIGHT / 2.0
            pixel_error_x = cx - center_x
            pixel_error_y = cy - center_y

            # ---- 5. 3D Target Localization ----
            rel_pos_body, target_pos_world_est, depth_est = self.transformer.localize_target_3d(
                cx, cy, radius_px, self.camera_mgr.K, self.data, self.camera_id, self.uav_body_id
            )
        else:
            target_pos_world_est = target_pos_world.copy()

        # ---- 6. Closed-Loop Controller ----
        ctrl_out = self.controller.compute_control(
            tracker_info, rel_pos_body, target_pos_world_est, self.data, dt
        )

        # ---- 7. Apply Actuator Forces ----
        self.controller.apply_control(
            self.model, self.data, ctrl_out, self.rotor_sites, self.uav_body_id
        )

        # ---- 8. Step Physics ----
        mujoco.mj_step(self.model, self.data)

        # ---- 9. HUD Display (mirrors PyBullet OpenCV camera window) ----
        key = self.hud.render(
            rgb_frame=rgb_frame,
            tracker_info=tracker_info,
            ctrl_out=ctrl_out,
            pixel_error_x=pixel_error_x,
            pixel_error_y=pixel_error_y,
            sim_time=self.data.time,
            keyboard_mode=keyboard_mode,
        )
        self._last_key = key

        # ---- 10. Keyboard Target Control (matches PyBullet W/S/A/D/R/F) ----
        if keyboard_mode and self._manual_target_pos is not None and key not in (-1, 255):
            delta = parse_keyboard_target_delta(key, target_move_speed, dt)
            self._manual_target_pos += delta
            # Safety: keep target above ground
            self._manual_target_pos[2] = max(self._manual_target_pos[2], 0.15)

        # ---- 11. Telemetry Logging ----
        uav_pos = self.data.qpos[0:3].copy()
        self.logger.log_step(
            t=self.data.time,
            target_pos=target_pos_world,
            uav_pos=uav_pos,
            rel_pos=rel_pos_body if rel_pos_body is not None else np.zeros(3),
            pixel_error_x=pixel_error_x,
            pixel_error_y=pixel_error_y,
            distance_error=ctrl_out["distance_error"],
            yaw_error=ctrl_out["yaw_error"],
            vx=ctrl_out["des_vx"],
            vy=ctrl_out["des_vy"],
            vz=ctrl_out["des_vz"],
            yaw_rate=ctrl_out["des_yaw_rate"],
            target_detected=tracker_info["found"],
            tracking_confidence=tracker_info.get("confidence", 0.0),
            tracking_state=tracker_info["state"]
        )

        return ctrl_out, tracker_info

    def _print_status(self, tracker_info: dict, ctrl_out: dict):
        """Prints periodic console status (matches PyBullet DEBUG print block)."""
        uav_pos = self.data.qpos[0:3]
        tgt_pos = (
            self._manual_target_pos
            if self._manual_target_pos is not None
            else self.target_mgr.trajectory.get_position(self.data.time)
        )
        st = tracker_info["state"]
        det = "YES" if tracker_info["found"] else "NO"
        print(
            f"T: {self.data.time:5.1f}s | State: {st:<8} | Det: {det} | "
            f"UAV: ({uav_pos[0]:4.1f},{uav_pos[1]:4.1f},{uav_pos[2]:4.1f}) | "
            f"TGT: ({tgt_pos[0]:4.1f},{tgt_pos[1]:4.1f},{tgt_pos[2]:4.1f}) | "
            f"DistErr: {ctrl_out['distance_error']:4.2f}m | "
            f"YawErr: {np.degrees(ctrl_out['yaw_error']):+4.1f}°"
        )
