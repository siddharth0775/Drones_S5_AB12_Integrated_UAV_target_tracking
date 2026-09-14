import sys
import mujoco
import mujoco.viewer
import numpy as np

# ============================================================
# 1. IMPORT EXISTING TRACKER & TRACKING CONTROLLER
#    (DO NOT MODIFY camera_tracker.py)
# ============================================================

sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker, TrackingController


# ============================================================
# 2. LOAD MUJOCO MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path("models/uav_target.xml")
data = mujoco.MjData(model)


# ============================================================
# 3. IDENTIFY MODEL BODIES, CAMERA & ROTOR SITES
# ============================================================

uav_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "uav")
target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "uav_camera")

rotor_sites = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thrust_{i}")
    for i in range(1, 5)
]

print("============================================================")
print(" MUJOCO INTEGRATED UAV CLOSED-LOOP TARGET TRACKING SYSTEM")
print("============================================================")
print(f"UAV body ID: {uav_body}")
print(f"Target body ID: {target_body}")
print(f"Camera ID: {camera_id}")
print(f"Rotor sites: {rotor_sites}")
print("============================================================\n")


# ============================================================
# 4. CAMERA SENSOR RENDERER
# ============================================================

WIDTH = 320
HEIGHT = 240

renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)


# ============================================================
# 5. VISION TRACKER & HIGH-LEVEL TRACKING CONTROLLER
# ============================================================

tracker = ColorBlobTracker()
controller = TrackingController(target_radius_px=30)


# ============================================================
# 6. UAV PHYSICAL & CONTROLLER PARAMETERS
# ============================================================

mass = 1.0
gravity = 9.81

# Altitude (Z) control
target_altitude = 3.0
Kp_z = 15.0
Kd_z = 6.0

# Attitude (Roll & Pitch) control
Kp_roll = 2.0
Kd_roll = 0.5

Kp_pitch = 2.0
Kd_pitch = 0.5

# Yaw attitude control (integrates desired yaw_rate like reference)
Kp_yaw = 1.5
Kd_yaw = 0.4
yaw_torque_scale = 0.15

commanded_yaw = 0.0
last_time = 0.0
last_print_time = 0.0


# ============================================================
# 7. MAIN SIMULATION LOOP (CLOSED-LOOP VISUAL SERVOING)
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        t = data.time

        # --------------------------------------------------------
        # TARGET TRAJECTORY (3D Dynamic Motion)
        # --------------------------------------------------------
        target_x = 4.0 + 0.3 * np.sin(0.4 * t)
        target_y = 0.6 * np.sin(0.5 * t)
        target_z = 3.05 + 0.25 * np.cos(0.3 * t)

        model.body_pos[target_body] = np.array([target_x, target_y, target_z])

        # Forward kinematics
        mujoco.mj_forward(model, data)

        # --------------------------------------------------------
        # CAMERA SENSOR & TARGET DETECTION (Vision Layer)
        # --------------------------------------------------------
        renderer.update_scene(data, camera=camera_id)
        image = renderer.render()

        cx, cy, radius, found = tracker.find_target(image)

        # --------------------------------------------------------
        # DT CALCULATION
        # --------------------------------------------------------
        dt = t - last_time
        if dt <= 0:
            dt = model.opt.timestep
        last_time = t

        # --------------------------------------------------------
        # RUN EXISTING TrackingController
        # --------------------------------------------------------
        commands = controller.update(
            cx=cx if cx is not None else 0,
            cy=cy if cy is not None else 0,
            radius_px=radius if radius is not None else 0,
            frame_w=WIDTH,
            frame_h=HEIGHT,
            found=found,
            dt=dt
        )

        # High-level motion commands from tracker
        des_vx = commands["vx"]
        des_vz = commands["vz"]
        des_yaw_rate = commands["yaw_rate"]

        # When target is actively tracked:
        if found:
            # 1. Integrate yaw setpoint (matches reference PyBullet architecture)
            commanded_yaw += des_yaw_rate * dt
            commanded_yaw = (commanded_yaw + np.pi) % (2.0 * np.pi) - np.pi

            # 2. Adjust target altitude from vertical visual error
            target_altitude += des_vz * dt
            target_altitude = np.clip(target_altitude, 1.2, 5.0)
        else:
            # When target lost, hold forward speed
            des_vx = 0.0

        # --------------------------------------------------------
        # UAV ORIENTATION & VELOCITIES
        # --------------------------------------------------------
        quat = data.qpos[3:7]
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, quat)
        rot = rot.reshape(3, 3)

        # Euler angles
        cur_roll = np.arctan2(rot[2, 1], rot[2, 2])
        cur_pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2))
        cur_yaw = np.arctan2(rot[1, 0], rot[0, 0])

        roll_vel = data.qvel[3]
        pitch_vel = data.qvel[4]
        cur_yaw_rate = data.qvel[5]

        # Current UAV position and vertical velocity
        x = data.qpos[0]
        y = data.qpos[1]
        z = data.qpos[2]
        current_vz = data.qvel[2]

        # --------------------------------------------------------
        # 1. YAW CONTROL (Closed-Loop Attitude Alignment)
        # --------------------------------------------------------
        yaw_err = (commanded_yaw - cur_yaw + np.pi) % (2.0 * np.pi) - np.pi
        tau_yaw = np.clip(
            Kp_yaw * yaw_err - Kd_yaw * cur_yaw_rate,
            -yaw_torque_scale,
            yaw_torque_scale
        )

        # --------------------------------------------------------
        # 2. FORWARD TRANSLATION CONTROL (vx -> pitch)
        # --------------------------------------------------------
        cur_vx_body = rot[:, 0] @ data.qvel[0:3]
        vel_err = des_vx - cur_vx_body
        pitch_cmd = np.clip(0.8 * vel_err, -0.8, 0.8)
        tgt_pitch = np.radians(10.0 * pitch_cmd)

        pitch_err = tgt_pitch - cur_pitch
        pitch_control = Kp_pitch * pitch_err - Kd_pitch * pitch_vel

        # --------------------------------------------------------
        # 3. ROLL STABILIZATION
        # --------------------------------------------------------
        tgt_roll = 0.0
        roll_err = tgt_roll - cur_roll
        roll_control = Kp_roll * roll_err - Kd_roll * roll_vel

        # --------------------------------------------------------
        # 4. ALTITUDE CONTROL
        # --------------------------------------------------------
        altitude_error = target_altitude - z
        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * current_vz
        )
        total_thrust = max(total_thrust, 0.0)

        # --------------------------------------------------------
        # 5. MOTOR MIXER (Four Rotors)
        # --------------------------------------------------------
        base_thrust = total_thrust / 4.0

        motor_1 = max(base_thrust + roll_control - pitch_control, 0.0)
        motor_2 = max(base_thrust - roll_control - pitch_control, 0.0)
        motor_3 = max(base_thrust - roll_control + pitch_control, 0.0)
        motor_4 = max(base_thrust + roll_control + pitch_control, 0.0)

        motor_commands = [motor_1, motor_2, motor_3, motor_4]

        # --------------------------------------------------------
        # 6. APPLY FORCES & TORQUES TO MUJOCO
        # --------------------------------------------------------
        data.qfrc_applied[:] = 0.0

        # Apply rotor thrusts
        for site_id, thrust in zip(rotor_sites, motor_commands):
            point = data.site_xpos[site_id].copy()
            force = rot @ np.array([0.0, 0.0, thrust])
            mujoco.mj_applyFT(
                model, data, force, np.zeros(3), point, uav_body, data.qfrc_applied
            )

        # Apply controlled yaw torque to UAV Center of Mass
        world_yaw_torque = rot @ np.array([0.0, 0.0, tau_yaw])
        mujoco.mj_applyFT(
            model, data, np.zeros(3), world_yaw_torque, data.xpos[uav_body].copy(), uav_body, data.qfrc_applied
        )

        # Step physics
        mujoco.mj_step(model, data)

        # --------------------------------------------------------
        # PRINT TELEMETRY (1 Hz)
        # --------------------------------------------------------
        if t - last_print_time >= 1.0:
            cx_str = f"{cx:5.1f}" if cx is not None else " None"
            cy_str = f"{cy:5.1f}" if cy is not None else " None"
            r_str = f"{radius:4.1f}" if radius is not None else " None"

            print(
                f"Time: {t:5.1f}s | "
                f"UAV: ({x:4.2f}, {y:4.2f}, {z:4.2f}) | "
                f"TGT: ({target_x:4.2f}, {target_y:4.2f}, {target_z:4.2f}) | "
                f"CX: {cx_str} | "
                f"CY: {cy_str} | "
                f"Rad: {r_str} | "
                f"Yaw: {np.degrees(cur_yaw):+5.1f}° | "
                f"Vx: {des_vx:4.2f} | "
                f"State: {controller.state}"
            )
            last_print_time = t

        viewer.sync()

renderer.close()
print("Simulation finished.")
