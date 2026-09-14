import sys
import mujoco
import numpy as np


# ---------------------------------------------------------
# Add existing PyBullet source folder
# ---------------------------------------------------------

sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker, TrackingController


# ---------------------------------------------------------
# Load MuJoCo model
# ---------------------------------------------------------

model = mujoco.MjModel.from_xml_path(
    "models/uav_target.xml"
)

data = mujoco.MjData(model)


# ---------------------------------------------------------
# Find camera and target
# ---------------------------------------------------------

camera_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_CAMERA,
    "uav_camera"
)

target_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "target"
)


print("Camera ID:", camera_id)
print("Target body ID:", target_body)


# ---------------------------------------------------------
# Camera settings
# ---------------------------------------------------------

WIDTH = 320
HEIGHT = 240

renderer = mujoco.Renderer(
    model,
    height=HEIGHT,
    width=WIDTH
)


# ---------------------------------------------------------
# Create YOUR existing vision tracker
# ---------------------------------------------------------

tracker = ColorBlobTracker()


# ---------------------------------------------------------
# Create YOUR existing tracking controller
# ---------------------------------------------------------

controller = TrackingController(
    target_radius_px=30
)


# ---------------------------------------------------------
# Keep UAV fixed
# ---------------------------------------------------------

uav_position = np.array([
    0.0,
    0.0,
    2.0
])


# ---------------------------------------------------------
# Timing
# ---------------------------------------------------------

previous_time = data.time
last_print_time = -1.0


# ---------------------------------------------------------
# Simulation
# ---------------------------------------------------------

for step in range(5000):

    t = data.time

    # -----------------------------------------------------
    # Calculate dt
    # -----------------------------------------------------

    dt = t - previous_time

    if dt <= 0:
        dt = model.opt.timestep

    previous_time = t


    # -----------------------------------------------------
    # Keep UAV fixed
    # -----------------------------------------------------

    data.qpos[0:3] = uav_position

    data.qpos[3:7] = np.array([
        1.0,
        0.0,
        0.0,
        0.0
    ])

    data.qvel[:] = 0.0


    # -----------------------------------------------------
    # Move target
    #
    # First target moves sideways.
    # -----------------------------------------------------

    target_x = 2.5

    target_y = 1.5 * np.sin(0.5 * t)

    target_z = 2.5

    model.body_pos[target_body] = np.array([
        target_x,
        target_y,
        target_z
    ])


    # -----------------------------------------------------
    # Update MuJoCo
    # -----------------------------------------------------

    mujoco.mj_forward(
        model,
        data
    )


    # -----------------------------------------------------
    # Render camera image
    # -----------------------------------------------------

    renderer.update_scene(
        data,
        camera=camera_id
    )

    image = renderer.render()


    # -----------------------------------------------------
    # STEP 1:
    # Use YOUR existing ColorBlobTracker
    # -----------------------------------------------------

    cx, cy, radius, found = tracker.find_target(
        image
    )


    # -----------------------------------------------------
    # STEP 2:
    # Use YOUR existing TrackingController
    # -----------------------------------------------------

    commands = controller.update(
        cx=cx,
        cy=cy,
        radius_px=radius,
        frame_w=WIDTH,
        frame_h=HEIGHT,
        found=found,
        dt=dt
    )


    # -----------------------------------------------------
    # Print results once per second
    # -----------------------------------------------------

    if t - last_print_time >= 1.0:

        if found:

            print(
                f"Time: {t:6.2f} s | "
                f"CX: {cx:6.1f} | "
                f"CY: {cy:6.1f} | "
                f"Radius: {radius:6.1f} | "
                f"State: {controller.state:9s} | "
                f"vx: {commands['vx']:6.3f} | "
                f"vy: {commands['vy']:6.3f} | "
                f"vz: {commands['vz']:6.3f} | "
                f"yaw: {commands['yaw_rate']:6.3f}"
            )

        else:

            print(
                f"Time: {t:6.2f} s | "
                f"Target NOT FOUND | "
                f"State: {controller.state:9s} | "
                f"vx: {commands['vx']:6.3f} | "
                f"vy: {commands['vy']:6.3f} | "
                f"vz: {commands['vz']:6.3f} | "
                f"yaw: {commands['yaw_rate']:6.3f}"
            )

        last_print_time = t


    # -----------------------------------------------------
    # Step MuJoCo
    # -----------------------------------------------------

    mujoco.mj_step(
        model,
        data
    )


# ---------------------------------------------------------
# Finish
# ---------------------------------------------------------

renderer.close()

print("Tracking controller test finished.")