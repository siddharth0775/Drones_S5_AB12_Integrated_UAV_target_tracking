import mujoco
import mujoco.viewer
import numpy as np


# -------------------------------------------------
# LOAD MODEL
# -------------------------------------------------

model = mujoco.MjModel.from_xml_path("models/uav.xml")
data = mujoco.MjData(model)


# -------------------------------------------------
# UAV PARAMETERS
# -------------------------------------------------

mass = 1.0
gravity = 9.81

target_z = 3.0

# Altitude controller
Kp_z = 15.0
Kd_z = 6.0


# -------------------------------------------------
# YAW CONTROLLER
# -------------------------------------------------

target_yaw = 0.0

Kp_yaw = 1.0
Kd_yaw = 0.2

# Simplified rotor reaction-torque coefficient
yaw_torque_coeff = 0.02


# -------------------------------------------------
# FIND UAV BODY
# -------------------------------------------------

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)


# -------------------------------------------------
# FIND ROTOR SITES
# -------------------------------------------------

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


# -------------------------------------------------
# ROTOR SPIN DIRECTIONS
# -------------------------------------------------

# Alternating rotor directions
spin_dirs = [
    1,
    -1,
    1,
    -1
]


# -------------------------------------------------
# PRINT TIMER
# -------------------------------------------------

last_print_time = 0.0


# -------------------------------------------------
# START MUJOCO
# -------------------------------------------------

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # -------------------------------------------------
        # YAW TARGET
        # -------------------------------------------------

        if data.time < 2.0:
            target_yaw = 0.0
        else:
            target_yaw = np.radians(5.0)


        # -------------------------------------------------
        # ALTITUDE CONTROL
        # -------------------------------------------------

        current_z = data.qpos[2]

        vertical_velocity = data.qvel[2]

        altitude_error = target_z - current_z

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * vertical_velocity
        )

        total_thrust = max(total_thrust, 0.0)


        # -------------------------------------------------
        # UAV ROTATION MATRIX
        # -------------------------------------------------

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(
            rotation,
            quat
        )

        rotation = rotation.reshape(3, 3)


        # -------------------------------------------------
        # CURRENT YAW
        # -------------------------------------------------

        current_yaw = np.arctan2(
            rotation[1, 0],
            rotation[0, 0]
        )


        # -------------------------------------------------
        # YAW ERROR
        # -------------------------------------------------

        yaw_error = (
            target_yaw
            - current_yaw
            + np.pi
        ) % (2 * np.pi) - np.pi


        # -------------------------------------------------
        # YAW RATE
        # -------------------------------------------------

        yaw_velocity = data.qvel[5]


        # -------------------------------------------------
        # YAW CONTROL
        # -------------------------------------------------

        yaw_torque = (
            Kp_yaw * yaw_error
            - Kd_yaw * yaw_velocity
        )


        # -------------------------------------------------
        # CONVERT YAW TORQUE TO MOTOR DIFFERENCE
        # -------------------------------------------------

        yaw_control = (
            yaw_torque
            / (4.0 * yaw_torque_coeff)
        )


        # -------------------------------------------------
        # BASE THRUST
        # -------------------------------------------------

        base_thrust = total_thrust / 4.0


        # -------------------------------------------------
        # MOTOR MIXER
        # -------------------------------------------------

        motor_1 = base_thrust + yaw_control
        motor_2 = base_thrust - yaw_control
        motor_3 = base_thrust + yaw_control
        motor_4 = base_thrust - yaw_control


        # -------------------------------------------------
        # PREVENT NEGATIVE THRUST
        # -------------------------------------------------

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


        # -------------------------------------------------
        # CLEAR PREVIOUS FORCES
        # -------------------------------------------------

        data.qfrc_applied[:] = 0.0


        # -------------------------------------------------
        # APPLY THRUST + YAW REACTION TORQUE
        # -------------------------------------------------

        for i, (site_id, thrust) in enumerate(
            zip(rotor_sites, motor_commands)
        ):

            # Rotor position
            point = data.site_xpos[site_id].copy()


            # Thrust in UAV local Z direction
            local_force = np.array([
                0.0,
                0.0,
                thrust
            ])


            # Convert force to world coordinates
            force = rotation @ local_force


            # Rotor reaction torque
            local_torque = np.array([
                0.0,
                0.0,
                spin_dirs[i]
                * yaw_torque_coeff
                * thrust
            ])


            # Convert torque to world coordinates
            torque = rotation @ local_torque


            # Apply force + torque
            mujoco.mj_applyFT(
                model,
                data,
                force,
                torque,
                point,
                uav_body,
                data.qfrc_applied
            )


        # -------------------------------------------------
        # STEP SIMULATION
        # -------------------------------------------------

        mujoco.mj_step(
            model,
            data
        )


        # -------------------------------------------------
        # PRINT DATA EVERY 0.2 SECONDS
        # -------------------------------------------------

        if data.time - last_print_time >= 0.2:

            print(
                f"Time: {data.time:5.1f} s | "
                f"Altitude: {data.qpos[2]:5.2f} m | "
                f"Target yaw: "
                f"{np.degrees(target_yaw):6.2f} deg | "
                f"Yaw: "
                f"{np.degrees(current_yaw):7.2f} deg | "
                f"Yaw control: "
                f"{yaw_control:7.3f} | "
                f"Motors: "
                f"{motor_1:.2f}, "
                f"{motor_2:.2f}, "
                f"{motor_3:.2f}, "
                f"{motor_4:.2f}"
            )

            last_print_time = data.time


        # -------------------------------------------------
        # UPDATE VIEWER
        # -------------------------------------------------

        viewer.sync()
        viewer.sync()