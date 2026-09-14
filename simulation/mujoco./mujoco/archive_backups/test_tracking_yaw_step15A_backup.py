import sys
import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# IMPORT EXISTING TRACKER
# ============================================================

sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker, TrackingController


# ============================================================
# LOAD MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path(
    "models/uav_target.xml"
)

data = mujoco.MjData(model)


# ============================================================
# FIND BODIES / CAMERA
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
# ROTOR SITES
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
# CAMERA / RENDERER
# ============================================================

WIDTH = 320
HEIGHT = 240

renderer = mujoco.Renderer(
    model,
    height=HEIGHT,
    width=WIDTH
)


# ============================================================
# EXISTING TRACKER + CONTROLLER
# ============================================================

tracker = ColorBlobTracker()

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
# YAW CONTROLLER
# ============================================================

Kp_yaw = 1.0
Kd_yaw = 0.2

# Reduced yaw torque for a slower and more controllable response
yaw_torque_coeff = 0.002

# Rotor spin directions
spin_dirs = [
    1,
    -1,
    1,
    -1
]


# ============================================================
# TIMING
# ============================================================

last_time = 0.0
last_print_time = 0.0


# ============================================================
# HELPER: CURRENT YAW
# ============================================================

def get_yaw(rotation):

    yaw = np.arctan2(
        rotation[1, 0],
        rotation[0, 0]
    )

    return yaw


# ============================================================
# START SIMULATION
# ============================================================

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        # ----------------------------------------------------
        # TIME
        # ----------------------------------------------------

        t = data.time


        # ----------------------------------------------------
        # TARGET
        # ----------------------------------------------------

        target_x = 3.0
        target_y = 1.0
        target_z = 2.5

        model.body_pos[target_body] = np.array([
            target_x,
            target_y,
            target_z
        ])


        # ----------------------------------------------------
        # FORWARD MODEL UPDATE
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
        # TARGET DETECTION
        # ----------------------------------------------------

        cx, cy, radius, found = tracker.find_target(
            image
        )


        # ----------------------------------------------------
        # DT
        # ----------------------------------------------------

        dt = t - last_time

        if dt <= 0:
            dt = model.opt.timestep

        last_time = t


        # ----------------------------------------------------
        # TRACKING CONTROLLER
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
        # GET COMMANDS
        # ----------------------------------------------------

        desired_yaw_rate = commands["yaw_rate"]

        # Stop yaw command when target is lost
        if not found:
            desired_yaw_rate = 0.0

        # vx intentionally disabled for this test
        desired_vx = 0.0


        # ----------------------------------------------------
        # UAV POSITION
        # ----------------------------------------------------

        x = data.qpos[0]
        y = data.qpos[1]
        z = data.qpos[2]

        current_vz = data.qvel[2]


        # ----------------------------------------------------
        # ROTATION MATRIX
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
        # CURRENT YAW
        # ----------------------------------------------------

        current_yaw = get_yaw(
            rotation
        )


        # ----------------------------------------------------
        # CURRENT YAW RATE
        # ----------------------------------------------------

        current_yaw_rate = data.qvel[5]


        # ----------------------------------------------------
        # YAW-RATE CONTROLLER
        #
        # TrackingController gives the desired yaw rate.
        #
        # Kp term:
        #     produces yaw control from yaw-rate error
        #
        # Kd term:
        #     provides damping to slow the UAV rotation
        # ----------------------------------------------------

        yaw_rate_error = (
            desired_yaw_rate
            - current_yaw_rate
        )

        yaw_control = (
            Kp_yaw * yaw_rate_error
            - Kd_yaw * current_yaw_rate
        )


        # ----------------------------------------------------
        # LIMIT YAW CONTROL
        # ----------------------------------------------------

        yaw_control = np.clip(
            yaw_control,
            -0.5,
            0.5
        )


        # ----------------------------------------------------
        # ALTITUDE CONTROL
        # ----------------------------------------------------

        altitude_error = (
            target_altitude
            - z
        )

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
        # MOTOR BASE THRUST
        # ----------------------------------------------------

        base_thrust = (
            total_thrust
            / 4.0
        )


        # ----------------------------------------------------
        # YAW MOTOR MIXING
        #
        # Reversed direction based on the previous test.
        # ----------------------------------------------------

        motor_1 = (
            base_thrust
            - yaw_control
        )

        motor_2 = (
            base_thrust
            + yaw_control
        )

        motor_3 = (
            base_thrust
            - yaw_control
        )

        motor_4 = (
            base_thrust
            + yaw_control
        )


        # ----------------------------------------------------
        # PREVENT NEGATIVE THRUST
        # ----------------------------------------------------

        motor_commands = [
            max(motor_1, 0.0),
            max(motor_2, 0.0),
            max(motor_3, 0.0),
            max(motor_4, 0.0)
        ]


        # ----------------------------------------------------
        # CLEAR PREVIOUS FORCES
        # ----------------------------------------------------

        data.qfrc_applied[:] = 0.0


        # ----------------------------------------------------
        # APPLY THRUST + REACTION TORQUE
        # ----------------------------------------------------

        for i, (site_id, thrust) in enumerate(
            zip(
                rotor_sites,
                motor_commands
            )
        ):

            # Rotor position in world coordinates
            point = data.site_xpos[
                site_id
            ].copy()


            # ------------------------------------------------
            # THRUST
            #
            # Rotor thrust acts along UAV local +Z.
            # Convert it to world coordinates.
            # ------------------------------------------------

            force = rotation @ np.array([
                0.0,
                0.0,
                thrust
            ])


            # ------------------------------------------------
            # REACTION TORQUE
            #
            # Rotor spin direction determines yaw direction.
            # ------------------------------------------------

            local_torque = np.array([
                0.0,
                0.0,
                spin_dirs[i]
                * yaw_torque_coeff
                * thrust
            ])


            # Convert torque from local UAV frame
            # to world frame.

            torque = (
                rotation
                @ local_torque
            )


            # ------------------------------------------------
            # APPLY FORCE + TORQUE
            # ------------------------------------------------

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
        # STEP SIMULATION
        # ----------------------------------------------------

        mujoco.mj_step(
            model,
            data
        )


        # ----------------------------------------------------
        # PRINT RESULTS
        # ----------------------------------------------------

        if t - last_print_time >= 1.0:

            cx_text = (
                f"{cx:.2f}"
                if cx is not None
                else "None"
            )

            cy_text = (
                f"{cy:.2f}"
                if cy is not None
                else "None"
            )

            radius_text = (
                f"{radius:.2f}"
                if radius is not None
                else "None"
            )

            print(
                f"Time: {t:6.1f} s | "
                f"X: {x:6.2f} | "
                f"Y: {y:6.2f} | "
                f"Z: {z:5.2f} | "
                f"CX: {cx_text:>7} | "
                f"CY: {cy_text:>7} | "
                f"Radius: {radius_text:>6} | "
                f"Found: {found} | "
                f"State: {controller.state:9s} | "
                f"yaw_cmd: {desired_yaw_rate:6.3f} | "
                f"yaw: {np.degrees(current_yaw):6.2f}"
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

print("Tracking yaw test finished.")