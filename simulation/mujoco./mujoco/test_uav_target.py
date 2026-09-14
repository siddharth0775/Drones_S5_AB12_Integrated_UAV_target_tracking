import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# LOAD MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path(
    "models/uav_target.xml"
)

data = mujoco.MjData(model)


# ============================================================
# UAV PARAMETERS
# ============================================================

mass = 1.0
gravity = 9.81

target_z_uav = 3.0


# ============================================================
# ALTITUDE CONTROLLER
# ============================================================

Kp_z = 15.0
Kd_z = 6.0


# ============================================================
# ATTITUDE CONTROLLERS
# ============================================================

Kp_roll = 2.0
Kd_roll = 0.5

Kp_pitch = 2.0
Kd_pitch = 0.5

Kp_yaw = 1.0
Kd_yaw = 0.2


# ============================================================
# YAW PARAMETERS
# ============================================================

yaw_torque_coeff = 0.02

spin_dirs = [
    1,
    -1,
    1,
    -1
]


# ============================================================
# FIND UAV
# ============================================================

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)


# ============================================================
# FIND ROTOR SITES
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
# FIND TARGET
# ============================================================

target_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "target"
)


# ============================================================
# TARGET INITIAL POSITION
# ============================================================

target_x = 3.0
target_y = 0.0
target_z = 3.0


# ============================================================
# LOGGING
# ============================================================

last_print_time = 0.0


# ============================================================
# START VIEWER
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # ====================================================
        # CURRENT SIMULATION TIME
        # ====================================================

        t = data.time


        # ====================================================
        # MOVE TARGET
        # ====================================================

        target_x = 3.0 * np.sin(0.3 * t)

        target_y = 0.0

        target_z = 3.0


        model.body_pos[target_body] = [
            target_x,
            target_y,
            target_z
        ]


        # ====================================================
        # UAV ALTITUDE CONTROL
        # ====================================================

        current_z = data.qpos[2]

        vertical_velocity = data.qvel[2]

        altitude_error = (
            target_z_uav - current_z
        )

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * vertical_velocity
        )

        total_thrust = max(
            total_thrust,
            0.0
        )


        # ====================================================
        # UAV ROTATION MATRIX
        # ====================================================

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(
            rotation,
            quat
        )

        rotation = rotation.reshape(3, 3)


        # ====================================================
        # CURRENT ROLL
        # ====================================================

        current_roll = np.arctan2(
            rotation[2, 1],
            rotation[2, 2]
        )


        # ====================================================
        # CURRENT PITCH
        # ====================================================

        current_pitch = np.arctan2(
            -rotation[2, 0],
            np.sqrt(
                rotation[2, 1] ** 2
                + rotation[2, 2] ** 2
            )
        )


        # ====================================================
        # CURRENT YAW
        # ====================================================

        current_yaw = np.arctan2(
            rotation[1, 0],
            rotation[0, 0]
        )


        # ====================================================
        # TARGET ATTITUDE
        # ====================================================

        target_roll = 0.0
        target_pitch = 0.0
        target_yaw = 0.0


        # ====================================================
        # ANGULAR VELOCITIES
        # ====================================================

        roll_velocity = data.qvel[3]

        pitch_velocity = data.qvel[4]

        yaw_velocity = data.qvel[5]


        # ====================================================
        # ROLL CONTROL
        # ====================================================

        roll_error = (
            target_roll - current_roll
        )

        roll_control = (
            Kp_roll * roll_error
            - Kd_roll * roll_velocity
        )


        # ====================================================
        # PITCH CONTROL
        # ====================================================

        pitch_error = (
            target_pitch - current_pitch
        )

        pitch_control = (
            Kp_pitch * pitch_error
            - Kd_pitch * pitch_velocity
        )


        # ====================================================
        # YAW CONTROL
        # ====================================================

        yaw_error = (
            target_yaw
            - current_yaw
            + np.pi
        ) % (2 * np.pi) - np.pi

        yaw_torque = (
            Kp_yaw * yaw_error
            - Kd_yaw * yaw_velocity
        )

        yaw_control = (
            yaw_torque
            / (4 * yaw_torque_coeff)
        )


        # ====================================================
        # MOTOR MIXING
        # ====================================================

        base_thrust = (
            total_thrust / 4.0
        )


        motor_1 = (
            base_thrust
            + roll_control
            - pitch_control
            + yaw_control
        )

        motor_2 = (
            base_thrust
            - roll_control
            - pitch_control
            - yaw_control
        )

        motor_3 = (
            base_thrust
            - roll_control
            + pitch_control
            + yaw_control
        )

        motor_4 = (
            base_thrust
            + roll_control
            + pitch_control
            - yaw_control
        )


        # ====================================================
        # PREVENT NEGATIVE THRUST
        # ====================================================

        motor_1 = max(
            motor_1,
            0.0
        )

        motor_2 = max(
            motor_2,
            0.0
        )

        motor_3 = max(
            motor_3,
            0.0
        )

        motor_4 = max(
            motor_4,
            0.0
        )


        motor_commands = [
            motor_1,
            motor_2,
            motor_3,
            motor_4
        ]


        # ====================================================
        # CLEAR PREVIOUS FORCES
        # ====================================================

        data.qfrc_applied[:] = 0.0


        # ====================================================
        # APPLY ROTOR FORCES
        # ====================================================

        for i, (site_id, thrust) in enumerate(
            zip(rotor_sites, motor_commands)
        ):

            point = data.site_xpos[
                site_id
            ].copy()


            # Rotor thrust in UAV local +Z direction
            force = rotation @ np.array([
                0.0,
                0.0,
                thrust
            ])


            # Reaction torque for yaw
            local_torque = np.array([
                0.0,
                0.0,
                spin_dirs[i]
                * yaw_torque_coeff
                * thrust
            ])


            world_torque = (
                rotation @ local_torque
            )


            # Apply force + torque
            mujoco.mj_applyFT(
                model,
                data,
                force,
                world_torque,
                point,
                uav_body,
                data.qfrc_applied
            )


        # ====================================================
        # STEP SIMULATION
        # ====================================================

        mujoco.mj_step(
            model,
            data
        )


        # ====================================================
        # PRINT STATUS
        # ====================================================

        if (
            data.time - last_print_time
            >= 1.0
        ):

            print(
                f"Time: {data.time:6.1f} s | "
                f"UAV Z: {data.qpos[2]:6.2f} m | "
                f"Target X: {target_x:6.2f} m | "
                f"Target Y: {target_y:6.2f} m | "
                f"Target Z: {target_z:6.2f} m | "
                f"Roll: {np.degrees(current_roll):6.2f}° | "
                f"Pitch: {np.degrees(current_pitch):6.2f}° | "
                f"Yaw: {np.degrees(current_yaw):6.2f}°"
            )

            last_print_time = data.time


        # ====================================================
        # UPDATE VIEWER
        # ====================================================

        viewer.sync()