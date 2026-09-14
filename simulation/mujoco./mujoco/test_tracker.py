import sys
import mujoco
import numpy as np

from PIL import Image


# ---------------------------------------------------------
# Add existing PyBullet source folder to Python path
# ---------------------------------------------------------

sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker


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
# Create YOUR existing ColorBlobTracker
# ---------------------------------------------------------

tracker = ColorBlobTracker()


# ---------------------------------------------------------
# Keep UAV fixed for this test
# ---------------------------------------------------------

uav_position = np.array([
    0.0,
    0.0,
    2.0
])


saved_image = False


# ---------------------------------------------------------
# Simulation
# ---------------------------------------------------------

for step in range(5000):

    t = data.time


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
    # -----------------------------------------------------

    target_x = 2.0 + 0.5 * np.sin(0.5 * t)
    target_y = 0.0
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
    # SEND MUJOCO IMAGE TO YOUR EXISTING TRACKER
    # -----------------------------------------------------

    cx, cy, radius, found = tracker.find_target(
        image
    )


    # -----------------------------------------------------
    # Print tracking result
    # -----------------------------------------------------

    if step % 240 == 0:

        print(
            f"Time: {t:6.2f} s | "
            f"Target X: {target_x:5.2f} m | "
            f"Found: {found} | "
            f"CX: {cx} | "
            f"CY: {cy} | "
            f"Radius: {radius}"
        )


    # -----------------------------------------------------
    # Save camera image once
    # -----------------------------------------------------

    if t >= 2.0 and not saved_image:

        Image.fromarray(image).save(
            "tracker_test_image.png"
        )

        print(
            "Camera image saved: tracker_test_image.png"
        )

        saved_image = True


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

print("Tracker test finished.")