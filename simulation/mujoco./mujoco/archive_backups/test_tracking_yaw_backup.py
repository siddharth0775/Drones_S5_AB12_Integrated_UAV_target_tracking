import sys
import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# IMPORT EXISTING TRACKER AND TRACKING CONTROLLER
# ============================================================

sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker, TrackingController


# ============================================================
# LOAD MUJOCO MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path(
    "models/uav_target.xml"
)

data = mujoco.MjData(model)


# ============================================================
# FIND UAV, TARGET AND CAMERA
# ============================================================

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)

target_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "target"
)

camera_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_CAMERA,
    "uav_camera"
)

print("UAV body ID:", uav_body)
print("Target body ID:", target_body)
print("Camera ID:", camera_id)


# ============================================================
# FIND ROTORS
# ============================================================

rotor_sites = [
    mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "thrust_1"
    ),
    mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "thrust_2"
    ),
    mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "thrust_3"
    ),
    mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "thrust_4"
    )
]


# ============================================================
# CAMERA SETTINGS
# ============================================================

WIDTH = 320
HEIGHT = 240

renderer = mujoco.Renderer(
    model,
    height=HEIGHT,
    width=WIDTH
)


# ============================================================
# EXISTING TRACKER
# ============================================================

tracker = ColorBlobTracker()


# ============================================================
# EXISTING TRACKING CONTROLLER
# ============================================================

controller = TrackingController(
    target_radius_px=30
)


# ============================================================
# ALTITUDE CONTROLLER
# ============================================================

mass = 1.0
gravity = 9.81

target_altitude = 3.0

Kp_z = 15.0
Kd_z = 6.0


# ============================================================
# PITCH CONTROLLER
# ============================================================

Kp_pitch = 2.0
Kd_pitch = 0.5


# ============================================================
# TIMING
# ============================================================

last_time = 0.0
last_print_time = 0.0


# ============================================================
# START SIMULATION
# ============================================================

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        # ----------------------------------------------------
        # CURRENT TIME
        # ----------------------------------------------------

        t = data.time


        # ----------------------------------------------------
        # MOVE TARGET
        # ----------------------------------------------------
        #
        # Target stays directly in front of the UAV.
        # Only distance is being tested here.
        #

        target_x = 3.5
        target_y = 0.0
        target_z = 2.5

        model.body_pos[target_body] = np.array([
            target_x,
            target_y,
            target_z
        ])


        # ----------------------------------------------------
        # UPDATE MUJOCO
        # ----------------------------------------------------

        mujoco.mj_forward(
            model,
            data
        )


        # ----------------------------------------------------
        # RENDER CAMERA
        # ----------------------------------------------------

        renderer.update_scene(
            data,
            camera=camera_id
        )

        image = renderer.render()


        # ----------------------------------------------------
        # DETECT TARGET
        # ----------------------------------------------------

        cx, cy, radius, found = tracker.find_target(
            image
        )


        # ----------------------------------------------------
        # CALCULATE DT
        # ----------------------------------------------------

        dt = t - last_time

        if dt <= 0:
            dt = model.opt.timestep

        last_time = t


        # ----------------------------------------------------
        # RUN EXISTING TRACKING CONTROLLER
        # ----------------------------------------------------

        commands = controller.update(
            cx=cx,
            cy=cy,
            radius_px=radius,
            frame_w=WIDTH,
            frame_h=HEIGHT,
            found=found,
            dt=dt
        )


        # ----------------------------------------------------
        # GET VX COMMAND
        # ----------------------------------------------------

        desired_vx = commands["vx"]


        # ----------------------------------------------------
        # CURRENT UAV POSITION / VELOCITY
        # ----------------------------------------------------

        x = data.qpos[0]
        z = data.qpos[2]

        current_vx = data.qvel[0]
        current_vz = data.qvel[2]


        # ----------------------------------------------------
        # ALTITUDE CONTROL
        # ----------------------------------------------------

        altitude_error = target_altitude - z

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * current_vz
        )

        total_thrust = max(
            total_thrust,
            0.0
        )


        # ----------------------------------------------------
        # VX COMMAND → PITCH COMMAND
        # ----------------------------------------------------

        velocity_error = (
            desired_vx - current_vx
        )

        pitch_command = (
            0.8 * velocity_error
        )

        pitch_command = np.clip(
            pitch_command,
            -0.8,
            0.8
        )


        # Convert normalized command to degrees

        target_pitch = np.radians(
            10.0 * pitch_command
        )


        # ----------------------------------------------------
        # CURRENT ROTATION
        # ----------------------------------------------------

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(
            rotation,
            quat
        )

        rotation = rotation.reshape(
            3,
            3
        )


        # ----------------------------------------------------
        # CURRENT PITCH
        # ----------------------------------------------------

        current_pitch = np.arctan2(
            -rotation[2, 0],
            np.sqrt(
                rotation[2, 1] ** 2
                + rotation[2, 2] ** 2
            )
        )

        pitch_velocity = data.qvel[4]


        # ----------------------------------------------------
        # PITCH PID
        # ----------------------------------------------------

        pitch_error = (
            target_pitch - current_pitch
        )

        pitch_control = (
            Kp_pitch * pitch_error
            - Kd_pitch * pitch_velocity
        )


        # ----------------------------------------------------
        # MOTOR MIXER
        # ----------------------------------------------------

        base_thrust = total_thrust / 4.0

        motor_1 = base_thrust - pitch_control
        motor_2 = base_thrust - pitch_control
        motor_3 = base_thrust + pitch_control
        motor_4 = base_thrust + pitch_control


        # Prevent negative thrust

        motor_1 = max(motor_1, 0.0)
        motor_2 = max(motor_2, 0.0)
        motor_3 = max(motor_3, 0.0)
        motor_4 = max(motor_4, 0.0)


        motor_commands = [
            motor_1,
            motor_2,
            motor_3,
            motor_4
        ]


        # ----------------------------------------------------
        # APPLY MOTOR FORCES
        # ----------------------------------------------------

        data.qfrc_applied[:] = 0.0


        for site_id, thrust in zip(
            rotor_sites,
            motor_commands
        ):

            point = data.site_xpos[
                site_id
            ].copy()

            force = rotation @ np.array([
                0.0,
                0.0,
                thrust
            ])

            torque = np.array([
                0.0,
                0.0,
                0.0
            ])

            mujoco.mj_applyFT(
                model,
                data,
                force,
                torque,
                point,
                uav_body,
                data.qfrc_applied
            )


        # ----------------------------------------------------
        # STEP MUJOCO
        # ----------------------------------------------------

        mujoco.mj_step(
            model,
            data
        )


        # ----------------------------------------------------
        # PRINT RESULTS
        # ----------------------------------------------------

        if t - last_print_time >= 1.0:

            print(
                f"Time: {t:6.1f} s | "
                f"X: {x:6.2f} m | "
                f"Z: {z:5.2f} m | "
                f"CX: {cx:7.2f} | "
                f"CY: {cy:7.2f} | "
                f"Radius: {radius:6.2f} | "
                f"Found: {found} | "
                f"State: {controller.state:9s} | "
                f"vx_cmd: {desired_vx:6.3f}"
            )

            last_print_time = t


        # ----------------------------------------------------
        # UPDATE VIEWER
        # ----------------------------------------------------

        viewer.sync()


# ============================================================
# CLEANUP
# ============================================================

renderer.close()

print("Tracking vx test finished.")