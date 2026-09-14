import mujoco
import mujoco.viewer
import numpy as np

# ============================================================
# LOAD MUJOCO MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path("models/uav.xml")
data = mujoco.MjData(model)


# ============================================================
# UAV PARAMETERS
# ============================================================

mass = 1.0
gravity = 9.81

target_z = 3.0

# Altitude controller
Kp_z = 15.0
Kd_z = 6.0


# ============================================================
# ROLL CONTROLLER
# ============================================================

target_roll = 0.0

Kp_roll = 2.0
Kd_roll = 0.5


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
# INFORMATION
# ============================================================

print("========================================")
print("       MUJOCO UAV ROLL TEST")
print("========================================")
print("Position DOFs:", model.nq)
print("Velocity DOFs:", model.nv)
print("UAV body ID:", uav_body)
print("Rotor site IDs:", rotor_sites)
print("Target altitude:", target_z)
print("Target roll:", target_roll)
print("========================================")


last_print_time = 0.0


# ============================================================
# SIMULATION
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        if data.time < 2.0:
            target_roll = 0.0
        else:
            target_roll = np.radians(5.0)

        # ----------------------------------------------------
        # CURRENT ALTITUDE
        # ----------------------------------------------------

        current_z = data.qpos[2]
        vertical_velocity = data.qvel[2]

        altitude_error = target_z - current_z


        # ----------------------------------------------------
        # ALTITUDE CONTROLLER
        # ----------------------------------------------------

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * vertical_velocity
        )

        total_thrust = max(total_thrust, 0.0)


        # ----------------------------------------------------
        # CURRENT ROLL
        # ----------------------------------------------------

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(rotation, quat)

        rotation = rotation.reshape(3, 3)

        # Roll angle from rotation matrix
        current_roll = np.arctan2(
            rotation[2, 1],
            rotation[2, 2]
        )

        # Approximate roll angular velocity
        roll_velocity = data.qvel[3]


        # ----------------------------------------------------
        # ROLL CONTROLLER
        # ----------------------------------------------------

        roll_error = target_roll - current_roll

        roll_control = (
            Kp_roll * roll_error
            - Kd_roll * roll_velocity
        )


        # ----------------------------------------------------
        # MOTOR MIXING
        # ----------------------------------------------------

        base_thrust = total_thrust / 4.0

        motor_1 = base_thrust + roll_control
        motor_2 = base_thrust - roll_control
        motor_3 = base_thrust - roll_control
        motor_4 = base_thrust + roll_control


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

            point = data.site_xpos[site_id].copy()

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
        # STEP SIMULATION
        # ----------------------------------------------------

        mujoco.mj_step(model, data)


        # ----------------------------------------------------
        # PRINT INFORMATION
        # ----------------------------------------------------

        if data.time - last_print_time >= 0.2:

            print(
            f"Time: {data.time:6.1f} s | "
            f"Altitude: {data.qpos[2]:6.2f} m | "
            f"Target roll: {np.degrees(target_roll):6.2f} deg | "
            f"Roll: {np.degrees(current_roll):7.2f} deg | "
            f"Roll control: {roll_control:7.3f} | "
            f"Motors: "
            f"{motor_1:.2f}, "
            f"{motor_2:.2f}, "
            f"{motor_3:.2f}, "
            f"{motor_4:.2f}")
            last_print_time = data.time


        viewer.sync()