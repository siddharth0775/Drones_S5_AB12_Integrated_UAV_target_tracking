import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# LOAD MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path("models/uav.xml")
data = mujoco.MjData(model)


# ============================================================
# UAV PARAMETERS
# ============================================================

mass = 1.0
gravity = 9.81

target_z = 3.0


# ============================================================
# ALTITUDE CONTROLLER
# ============================================================

Kp_z = 15.0
Kd_z = 6.0


# ============================================================
# ROLL CONTROLLER
# ============================================================

Kp_roll = 2.0
Kd_roll = 0.5


# ============================================================
# PITCH CONTROLLER
# ============================================================

Kp_pitch = 2.0
Kd_pitch = 0.5


# ============================================================
# YAW CONTROLLER
# ============================================================

Kp_yaw = 1.0
Kd_yaw = 0.2

yaw_torque_coeff = 0.02


# ============================================================
# TARGET ATTITUDE
# ============================================================

target_roll = 0.0
target_pitch = 0.0
target_yaw = 0.0


# ============================================================
# FIND UAV BODY
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
# ROTOR SPIN DIRECTIONS
# ============================================================

spin_dirs = [
    1,
    -1,
    1,
    -1
]


# ============================================================
# PRINT TIMER
# ============================================================

last_print_time = 0.0


# ============================================================
# START SIMULATION
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # ====================================================
        # TARGET COMMANDS
        # ====================================================

        if data.time < 2.0:

            target_roll = 0.0
            target_pitch = 0.0
            target_yaw = 0.0

        elif data.time < 4.0:

            target_roll = np.radians(5.0)
            target_pitch = 0.0
            target_yaw = 0.0

        elif data.time < 6.0:

            target_roll = np.radians(5.0)
            target_pitch = np.radians(5.0)
            target_yaw = 0.0

        elif data.time < 8.0:

            target_roll = np.radians(5.0)
            target_pitch = np.radians(5.0)
            target_yaw = np.radians(5.0)

        else:

            target_roll = np.radians(5.0)
            target_pitch = np.radians(5.0)
            target_yaw = np.radians(5.0)


        # ====================================================
        # ALTITUDE CONTROL
        # ====================================================

        current_z = data.qpos[2]

        vertical_velocity = data.qvel[2]

        altitude_error = target_z - current_z

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
        # ROTATION MATRIX
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
        # ANGULAR VELOCITIES
        # ====================================================

        roll_velocity = data.qvel[3]

        pitch_velocity = data.qvel[4]

        yaw_velocity = data.qvel[5]


        # ====================================================
        # ROLL CONTROL
        # ====================================================

        roll_error = (
            target_roll
            - current_roll
        )

        roll_control = (
            Kp_roll * roll_error
            - Kd_roll * roll_velocity
        )


        # ====================================================
        # PITCH CONTROL
        # ====================================================

        pitch_error = (
            target_pitch
            - current_pitch
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
            / (4.0 * yaw_torque_coeff)
        )


        # ====================================================
        # BASE THRUST
        # ====================================================

        base_thrust = (
            total_thrust / 4.0
        )


        # ====================================================
        # FOUR-ROTOR MIXER
        # ====================================================

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
        # LIMIT MOTOR THRUST
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
        # APPLY MOTOR FORCES AND YAW TORQUES
        # ====================================================

        for i, (site_id, thrust) in enumerate(
            zip(
                rotor_sites,
                motor_commands
            )
        ):

            point = data.site_xpos[
                site_id
            ].copy()


            # -----------------------------
            # THRUST
            # -----------------------------

            local_force = np.array([
                0.0,
                0.0,
                thrust
            ])

            force = (
                rotation
                @ local_force
            )


            # -----------------------------
            # ROTOR REACTION TORQUE
            # -----------------------------

            local_torque = np.array([
                0.0,
                0.0,
                spin_dirs[i]
                * yaw_torque_coeff
                * thrust
            ])

            torque = (
                rotation
                @ local_torque
            )


            # -----------------------------
            # APPLY TO UAV
            # -----------------------------

            mujoco.mj_applyFT(
                model,
                data,
                force,
                torque,
                point,
                uav_body,
                data.qfrc_applied
            )


        # ====================================================
        # STEP MUJOCO
        # ====================================================

        mujoco.mj_step(
            model,
            data
        )


        # ====================================================
        # PRINT EVERY 0.2 SECONDS
        # ====================================================

        if (
            data.time
            - last_print_time
            >= 0.2
        ):

            print(
                f"Time: {data.time:5.1f} s | "
                f"Alt: {data.qpos[2]:5.2f} m | "

                f"Roll: "
                f"{np.degrees(current_roll):6.2f}° / "
                f"{np.degrees(target_roll):5.1f}° | "

                f"Pitch: "
                f"{np.degrees(current_pitch):6.2f}° / "
                f"{np.degrees(target_pitch):5.1f}° | "

                f"Yaw: "
                f"{np.degrees(current_yaw):6.2f}° / "
                f"{np.degrees(target_yaw):5.1f}° | "

                f"Motors: "
                f"{motor_1:.2f}, "
                f"{motor_2:.2f}, "
                f"{motor_3:.2f}, "
                f"{motor_4:.2f}"
            )

            last_print_time = data.time


        # ====================================================
        # UPDATE VIEWER
        # ====================================================

        viewer.sync()