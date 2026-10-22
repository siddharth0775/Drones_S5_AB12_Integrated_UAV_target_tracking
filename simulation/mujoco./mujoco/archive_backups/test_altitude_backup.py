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

# Desired altitude
target_z = 3.0

# PD controller gains
Kp = 15.0
Kd = 6.0


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
# PRINT INITIAL INFORMATION
# ============================================================

print("========================================")
print("       MUJOCO UAV SIMULATION")
print("========================================")

print("Number of position DOFs:", model.nq)
print("Number of velocity DOFs:", model.nv)

print("UAV body ID:", uav_body)
print("Rotor site IDs:", rotor_sites)

print("Target altitude:", target_z, "m")

print("========================================")


# ============================================================
# VARIABLE FOR PERIODIC PRINTING
# ============================================================

last_print_time = 0.0


# ============================================================
# START SIMULATION
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # ----------------------------------------------------
        # CURRENT UAV STATE
        # ----------------------------------------------------

        current_z = data.qpos[2]

        vertical_velocity = data.qvel[2]


        # ----------------------------------------------------
        # ALTITUDE ERROR
        # ----------------------------------------------------

        altitude_error = target_z - current_z


        # ----------------------------------------------------
        # PD ALTITUDE CONTROLLER
        # ----------------------------------------------------

        total_thrust = (
            mass * gravity
            + Kp * altitude_error
            - Kd * vertical_velocity
        )


        # ----------------------------------------------------
        # LIMIT THRUST
        # ----------------------------------------------------

        if total_thrust < 0:
            total_thrust = 0.0


        # ----------------------------------------------------
        # MOTOR MIXING
        #
        # For now all four motors receive equal thrust.
        # ----------------------------------------------------

        motor_1 = total_thrust / 4
        motor_2 = total_thrust / 4
        motor_3 = total_thrust / 4
        motor_4 = total_thrust / 4

        motor_commands = [
            motor_1,
            motor_2,
            motor_3,
            motor_4
        ]


        # ----------------------------------------------------
        # CLEAR PREVIOUS FORCES
        # ----------------------------------------------------

        data.qfrc_applied[:] = 0


        # ----------------------------------------------------
        # UAV ORIENTATION
        # ----------------------------------------------------

        rotation = data.xmat[uav_body].reshape(3, 3)


        # ----------------------------------------------------
        # APPLY EACH MOTOR THRUST
        # ----------------------------------------------------

        for site_id, thrust in zip(
            rotor_sites,
            motor_commands
        ):

            # Rotor position in world coordinates
            point = data.site_xpos[site_id].copy()


            # Thrust direction
            #
            # Local +Z axis of UAV
            #
            force = rotation @ np.array([
                0.0,
                0.0,
                thrust
            ])


            # No motor reaction torque yet
            torque = np.array([
                0.0,
                0.0,
                0.0
            ])


            # Apply force at rotor position
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
        # ADVANCE PHYSICS
        # ----------------------------------------------------

        mujoco.mj_step(model, data)


        # ----------------------------------------------------
        # PRINT ALTITUDE EVERY 1 SECOND
        # ----------------------------------------------------

        if data.time - last_print_time >= 1.0:

            print(
                f"Time: {data.time:6.1f} s | "
                f"Altitude: {data.qpos[2]:6.2f} m | "
                f"Vertical velocity: {data.qvel[2]:6.2f} m/s | "
                f"Total thrust: {total_thrust:6.2f} N"
            )

            last_print_time = data.time


        # ----------------------------------------------------
        # UPDATE MUJOCO VIEWER
        # ----------------------------------------------------

        viewer.sync()