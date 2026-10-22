import mujoco
import mujoco.viewer
import numpy as np


# ========================================
# LOAD MUJOCO MODEL
# ========================================

model = mujoco.MjModel.from_xml_path("models/uav.xml")
data = mujoco.MjData(model)


# ========================================
# UAV PARAMETERS
# ========================================

mass = 1.0
gravity = 9.81

target_z = 3.0


# ========================================
# ALTITUDE PD CONTROLLER
# ========================================

Kp_z = 15.0
Kd_z = 6.0


# ========================================
# ROLL PD CONTROLLER
# ========================================

target_roll = 0.0

Kp_roll = 2.0
Kd_roll = 0.5


# ========================================
# PITCH PD CONTROLLER
# ========================================

target_pitch = 0.0

Kp_pitch = 2.0
Kd_pitch = 0.5


# ========================================
# FIND UAV BODY
# ========================================

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)


# ========================================
# FIND ROTOR SITES
# ========================================

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


# ========================================
# PRINT INFORMATION
# ========================================

print("========================================")
print("       MUJOCO UAV PITCH TEST")
print("========================================")

print("Position DOFs:", model.nq)
print("Velocity DOFs:", model.nv)
print("UAV body ID:", uav_body)
print("Rotor site IDs:", rotor_sites)

print("Target altitude:", target_z)
print("Target roll:", np.degrees(target_roll))
print("Target pitch:", np.degrees(target_pitch))

print("========================================")


last_print_time = 0.0


# ========================================
# START SIMULATION
# ========================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # ========================================
        # TARGET COMMANDS
        # ========================================

        # Keep roll at 0 degrees
        target_roll = 0.0

        # Keep pitch at 0 degrees initially
        # Change to 5 degrees after 2 seconds
        if data.time < 2.0:
            target_pitch = 0.0
        else:
            target_pitch = np.radians(5.0)


        # ========================================
        # ALTITUDE CONTROL
        # ========================================

        current_z = data.qpos[2]

        vertical_velocity = data.qvel[2]

        altitude_error = target_z - current_z

        total_thrust = (
            mass * gravity
            + Kp_z * altitude_error
            - Kd_z * vertical_velocity
        )

        total_thrust = max(total_thrust, 0.0)


        # ========================================
        # GET UAV ROTATION
        # ========================================

        quat = data.qpos[3:7]

        rotation = np.zeros(9)

        mujoco.mju_quat2Mat(
            rotation,
            quat
        )

        rotation = rotation.reshape(3, 3)


        # ========================================
        # CALCULATE ROLL
        # ========================================

        current_roll = np.arctan2(
            rotation[2, 1],
            rotation[2, 2]
        )

        roll_velocity = data.qvel[3]

        roll_error = target_roll - current_roll

        roll_control = (
            Kp_roll * roll_error
            - Kd_roll * roll_velocity
        )


        # ========================================
        # CALCULATE PITCH
        # ========================================

        current_pitch = np.arctan2(
            -rotation[2, 0],
            np.sqrt(
                rotation[2, 1] ** 2
                + rotation[2, 2] ** 2
            )
        )

        pitch_velocity = data.qvel[4]

        pitch_error = target_pitch - current_pitch

        pitch_control = (
            Kp_pitch * pitch_error
            - Kd_pitch * pitch_velocity
        )


        # ========================================
        # BASE THRUST
        # ========================================

        base_thrust = total_thrust / 4.0


        # ========================================
        # MOTOR MIXER
        # ========================================

        motor_1 = (
            base_thrust
            + roll_control
            - pitch_control
        )

        motor_2 = (
            base_thrust
            - roll_control
            - pitch_control
        )

        motor_3 = (
            base_thrust
            - roll_control
            + pitch_control
        )

        motor_4 = (
            base_thrust
            + roll_control
            + pitch_control
        )


        # ========================================
        # PREVENT NEGATIVE THRUST
        # ========================================

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


        # ========================================
        # CLEAR PREVIOUS FORCES
        # ========================================

        data.qfrc_applied[:] = 0.0


        # ========================================
        # APPLY MOTOR THRUST
        # ========================================

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


        # ========================================
        # STEP SIMULATION
        # ========================================

        mujoco.mj_step(
            model,
            data
        )


        # ========================================
        # PRINT EVERY 0.2 SECOND
        # ========================================

        if data.time - last_print_time >= 0.2:

            print(
                f"Time: {data.time:6.1f} s | "
                f"Altitude: {data.qpos[2]:6.2f} m | "
                f"Target pitch: "
                f"{np.degrees(target_pitch):6.2f} deg | "
                f"Pitch: "
                f"{np.degrees(current_pitch):7.2f} deg | "
                f"Pitch control: "
                f"{pitch_control:7.3f} | "
                f"Motors: "
                f"{motor_1:.2f}, "
                f"{motor_2:.2f}, "
                f"{motor_3:.2f}, "
                f"{motor_4:.2f}"
            )

            last_print_time = data.time


        # ========================================
        # UPDATE VIEWER
        # ========================================

        viewer.sync()