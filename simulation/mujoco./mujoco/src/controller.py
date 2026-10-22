"""
controller.py

Closed-Loop Visual Servoing & Low-Level Quadrotor Controller.
Generates rotor forces and yaw moments based on estimated 3D relative target position,
tracking errors, and quadrotor dynamics.
"""

from typing import Dict, Any
import numpy as np
import mujoco
from configs.config import SimConfig

class PID:
    """Generic PID controller with anti-windup and derivative filtering."""
    def __init__(self, kp: float, ki: float, kd: float, limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        if self.limit is not None:
            self.integral = float(np.clip(self.integral, -self.limit, self.limit))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        a = 0.25
        self.filtered_d = a * derivative + (1.0 - a) * self.filtered_d

        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_d
        if self.limit is not None:
            output = float(np.clip(output, -self.limit, self.limit))
        return output


class ClosedLoopUAVController:
    """Integrated High-Level Visual Controller & Low-Level Quadrotor Motor Controller."""

    def __init__(self, config=SimConfig):
        self.cfg = config

        # High-level tracking PIDs
        self.pid_vx = PID(config.KP_VX, 0.0, config.KD_VX, config.MAX_VX)
        self.pid_vy = PID(config.KP_VY, 0.0, config.KD_VY, config.MAX_VY)
        self.pid_z = PID(config.KP_Z, 0.0, config.KD_Z, config.MAX_VZ)

        # Yaw heading hold
        self.commanded_yaw = 0.0

    def reset(self):
        self.pid_vx.reset()
        self.pid_vy.reset()
        self.pid_z.reset()
        self.commanded_yaw = 0.0

    def compute_control(
        self,
        tracker_info: dict,
        rel_pos_body: np.ndarray,
        target_pos_world_est: np.ndarray,
        data: mujoco.MjData,
        dt: float
    ) -> Dict[str, Any]:
        """
        Computes velocity targets, tracking errors, and low-level rotor forces.
        """
        state = tracker_info.get("state", "LOST")
        found = tracker_info.get("found", False)

        # Extract current UAV state
        uav_pos = data.qpos[0:3].copy()
        quat = data.qpos[3:7].copy()
        
        rot_mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_mat, quat)
        rot_mat = rot_mat.reshape(3, 3)

        cur_roll = np.arctan2(rot_mat[2, 1], rot_mat[2, 2])
        cur_pitch = np.arctan2(-rot_mat[2, 0], np.sqrt(rot_mat[2, 1]**2 + rot_mat[2, 2]**2))
        cur_yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

        roll_rate = data.qvel[3]
        pitch_rate = data.qvel[4]
        yaw_rate = data.qvel[5]
        uav_vel_world = data.qvel[0:3].copy()
        uav_vel_body = rot_mat.T @ uav_vel_world

        des_vx = 0.0
        des_vy = 0.0
        des_vz = 0.0
        des_yaw_rate = 0.0

        pixel_error_x = 0.0
        pixel_error_y = 0.0
        distance_error = 0.0
        yaw_error = 0.0

        target_altitude = uav_pos[2]

        if found and rel_pos_body is not None:
            # ----------------------------------------------------
            # CLOSED-LOOP TRACKING STATE
            # ----------------------------------------------------
            # Target relative position in UAV body frame:
            # x_rel = forward distance, y_rel = lateral offset, z_rel = height offset
            x_rel, y_rel, z_rel = rel_pos_body

            # Compute tracking errors
            distance_error = x_rel - self.cfg.DESIRED_FOLLOWING_DISTANCE
            lateral_error = y_rel
            
            # Yaw alignment error to center target in field of view
            yaw_error = np.arctan2(y_rel, max(x_rel, 0.1))

            # Desired velocity commands
            des_vx = float(np.clip(self.pid_vx.update(distance_error, dt), -self.cfg.MAX_VX, self.cfg.MAX_VX))
            des_vy = float(np.clip(self.pid_vy.update(lateral_error, dt), -self.cfg.MAX_VY, self.cfg.MAX_VY))
            
            # Target Altitude tracking
            target_altitude = target_pos_world_est[2] + self.cfg.DESIRED_RELATIVE_HEIGHT
            target_altitude = np.clip(target_altitude, self.cfg.MIN_ALTITUDE, self.cfg.MAX_ALTITUDE)

            # Yaw rate setpoint
            des_yaw_rate = float(np.clip(self.cfg.KP_YAW * yaw_error, -self.cfg.MAX_YAW_RATE, self.cfg.MAX_YAW_RATE))
            self.commanded_yaw += des_yaw_rate * dt
            self.commanded_yaw = (self.commanded_yaw + np.pi) % (2.0 * np.pi) - np.pi

        elif state == "SEARCHING":
            # ----------------------------------------------------
            # TARGET TEMPORARILY LOST -> Controlled Search
            # ----------------------------------------------------
            search_dir = tracker_info.get("search_dir", 1.0)
            des_vx = 0.0
            des_vy = 0.0
            des_yaw_rate = 0.5 * search_dir
            self.commanded_yaw += des_yaw_rate * dt
            self.commanded_yaw = (self.commanded_yaw + np.pi) % (2.0 * np.pi) - np.pi
            target_altitude = uav_pos[2]

        else: # LOST
            # ----------------------------------------------------
            # TARGET LOST -> Scanning Spin / Hover Hold
            # ----------------------------------------------------
            des_vx = 0.0
            des_vy = 0.0
            des_yaw_rate = 0.25
            self.commanded_yaw += des_yaw_rate * dt
            self.commanded_yaw = (self.commanded_yaw + np.pi) % (2.0 * np.pi) - np.pi
            target_altitude = uav_pos[2]

        # Safety Altitude Bounds
        target_altitude = float(np.clip(target_altitude, self.cfg.MIN_ALTITUDE, self.cfg.MAX_ALTITUDE))

        # --------------------------------------------------------
        # LOW-LEVEL QUADROTOR CONTROLLER
        # --------------------------------------------------------
        # 1. Altitude PID -> Total Thrust
        alt_err = target_altitude - uav_pos[2]
        total_thrust = (
            self.cfg.MASS * self.cfg.GRAVITY
            + self.cfg.KP_Z * alt_err
            - self.cfg.KD_Z * uav_vel_world[2]
        )
        total_thrust = max(total_thrust, 0.0)

        # 2. Velocity to Attitude (Pitch & Roll setpoints)
        vel_err_x = des_vx - uav_vel_body[0]
        pitch_cmd = np.clip(0.8 * vel_err_x, -0.8, 0.8)
        target_pitch = np.radians(10.0 * pitch_cmd)

        vel_err_y = des_vy - uav_vel_body[1]
        roll_cmd = np.clip(-0.8 * vel_err_y, -0.8, 0.8)
        target_roll = np.radians(10.0 * roll_cmd)

        # Attitude PIDs
        pitch_err = target_pitch - cur_pitch
        pitch_control = self.cfg.KP_PITCH * pitch_err - self.cfg.KD_PITCH * pitch_rate

        roll_err = target_roll - cur_roll
        roll_control = self.cfg.KP_ROLL * roll_err - self.cfg.KD_ROLL * roll_rate

        # Yaw attitude torque control
        cur_yaw_err = (self.commanded_yaw - cur_yaw + np.pi) % (2.0 * np.pi) - np.pi
        tau_yaw = float(np.clip(
            self.cfg.KP_YAW * cur_yaw_err - self.cfg.KD_YAW * yaw_rate,
            -self.cfg.YAW_TORQUE_SCALE,
            self.cfg.YAW_TORQUE_SCALE
        ))

        # 3. Motor Mixing (4 Rotors)
        base_thrust = total_thrust / 4.0
        m1 = max(base_thrust + roll_control - pitch_control, 0.0)
        m2 = max(base_thrust - roll_control - pitch_control, 0.0)
        m3 = max(base_thrust - roll_control + pitch_control, 0.0)
        m4 = max(base_thrust + roll_control + pitch_control, 0.0)
        motor_commands = [m1, m2, m3, m4]

        return {
            "motor_commands": motor_commands,
            "tau_yaw": tau_yaw,
            "rot_mat": rot_mat,
            "des_vx": des_vx,
            "des_vy": des_vy,
            "des_vz": des_vz,
            "des_yaw_rate": des_yaw_rate,
            "distance_error": distance_error,
            "yaw_error": yaw_error,
            "target_altitude": target_altitude
        }

    def apply_control(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        control_output: dict,
        rotor_sites: list,
        uav_body_id: int
    ):
        """Applies rotor forces and yaw torque to MuJoCo simulation."""
        data.qfrc_applied[:] = 0.0
        rot_mat = control_output["rot_mat"]
        motor_commands = control_output["motor_commands"]
        tau_yaw = control_output["tau_yaw"]

        # Apply rotor thrusts at rotor site locations
        for site_id, thrust in zip(rotor_sites, motor_commands):
            point = data.site_xpos[site_id].copy()
            force = rot_mat @ np.array([0.0, 0.0, thrust])
            mujoco.mj_applyFT(model, data, force, np.zeros(3), point, uav_body_id, data.qfrc_applied)

        # Apply yaw moment around UAV CoM
        world_yaw_torque = rot_mat @ np.array([0.0, 0.0, tau_yaw])
        mujoco.mj_applyFT(model, data, np.zeros(3), world_yaw_torque, data.xpos[uav_body_id].copy(), uav_body_id, data.qfrc_applied)
