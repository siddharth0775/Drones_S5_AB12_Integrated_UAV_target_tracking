import mujoco
import mujoco.viewer
import numpy as np


# ==================================================
# LOAD MODEL
# ==================================================

model = mujoco.MjModel.from_xml_path("models/uav.xml")
data = mujoco.MjData(model)


# ==================================================
# UAV PARAMETERS
# ==================================================

mass = 1.0
gravity = 9.81

target_z = 3.0

Kp_z = 15.0
Kd_z = 6.0


# ==================================================
# HORIZONTAL MOTION
# ==================================================

target_x = 3.0

Kp_x = 2.0
Kd_x = 1.0


# ==================================================
# FIND UAV
# ==================================================

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)


# ==================================================
# FIND ROTOR SITES
# ==================================================

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


# ==================================================
# SIMULATION
# ==================================================

last_print_time = 0.0


with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # ------------------------------------------
        # CURRENT POSITION
        # ------------------------------------------

        x = data.qpos[0]
        z = data.qpos[2]

        vx = data.qvel[0]
        vz = data.qvel[2]


        # ------------------------------------------
        # ALTITUDE CONTROL
        # ------------------------------------------

        altitude_error = target_z - z

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * vz
        )

        total_thrust = max(total_thrust, 0.0)


        # ------------------------------------------
        # FORWARD POSITION CONTROL
        # ------------------------------------------

        x_error = target_x - x

        forward_control = (
            Kp_x * x_error
            - Kd_x * vx
        )


        # Limit forward control
        forward_control = np.clip(
            forward_control,
            -0.8,
            0.8
        )


        # ------------------------------------------
        # CONVERT FORWARD COMMAND
        # TO PITCH COMMAND
        # ------------------------------------------

        target_pitch = np.radians(
            10.0 * forward_control
        )


        # ------------------------------------------
        # ATTITUDE
        # ------------------------------------------

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(
            rotation,
            quat
        )

        rotation = rotation.reshape(3, 3)


        # Current pitch
        current_pitch = np.arctan2(
            -rotation[2, 0],
            np.sqrt(
                rotation[2, 1] ** 2
                + rotation[2, 2] ** 2
            )
        )


        pitch_velocity = data.qvel[4]


        # ------------------------------------------
        # PITCH CONTROLLER
        # ------------------------------------------

        Kp_pitch = 2.0
        Kd_pitch = 0.5

        pitch_error = (
            target_pitch
            - current_pitch
        )

        pitch_control = (
            Kp_pitch * pitch_error
            - Kd_pitch * pitch_velocity
        )


        # ------------------------------------------
        # MOTOR MIXER
        # ------------------------------------------

        base_thrust = total_thrust / 4.0


        motor_1 = base_thrust - pitch_control
        motor_2 = base_thrust - pitch_control
        motor_3 = base_thrust + pitch_control
        motor_4 = base_thrust + pitch_control


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


        # ------------------------------------------
        # APPLY FORCES
        # ------------------------------------------

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


        # ------------------------------------------
        # STEP
        # ------------------------------------------

        mujoco.mj_step(
            model,
            data
        )


        # ------------------------------------------
        # PRINT
        # ------------------------------------------

        if data.time - last_print_time >= 1.0:

            print(
                f"Time: {data.time:6.1f} s | "
                f"X: {x:6.2f} m | "
                f"Z: {z:6.2f} m | "
                f"Target X: {target_x:6.2f} m | "
                f"Pitch: "
                f"{np.degrees(current_pitch):7.2f} deg | "
                f"Pitch target: "
                f"{np.degrees(target_pitch):7.2f} deg"
            )

            last_print_time = data.time


        viewer.sync()