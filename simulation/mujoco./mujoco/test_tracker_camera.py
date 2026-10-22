import sys
import mujoco
import numpy as np
from PIL import Image

# Import existing tracker from PyBullet project
sys.path.append("../pybullet/src")

from camera_tracker import ColorBlobTracker


# --------------------------------------------------
# LOAD MUJOCO MODEL
# --------------------------------------------------

model = mujoco.MjModel.from_xml_path("models/uav_target.xml")
data = mujoco.MjData(model)


# --------------------------------------------------
# FIND OBJECTS
# --------------------------------------------------

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


# --------------------------------------------------
# CAMERA / IMAGE SETTINGS
# --------------------------------------------------

WIDTH = 320
HEIGHT = 240

renderer = mujoco.Renderer(
    model,
    height=HEIGHT,
    width=WIDTH
)


# --------------------------------------------------
# TRACKER
# --------------------------------------------------

tracker = ColorBlobTracker()


# --------------------------------------------------
# FIX UAV POSITION
# --------------------------------------------------

uav_position = np.array([
    0.0,
    0.0,
    2.0
])


saved_image = False

last_print_time = -1.0


# --------------------------------------------------
# SIMULATION LOOP
# --------------------------------------------------

for step in range(5000):

    t = data.time

    # Keep UAV fixed
    data.qpos[0:3] = uav_position
    data.qpos[3:7] = np.array([
        1.0,
        0.0,
        0.0,
        0.0
    ])

    data.qvel[:] = 0.0


    # --------------------------------------------------
    # MOVE TARGET
    # --------------------------------------------------

    target_x = 2.5 +0.8 *np.sin(0.5*t)

    target_y = 0.0

    target_z = 2.5

    model.body_pos[target_body] = np.array([
        target_x,
        target_y,
        target_z
    ])


    # Update MuJoCo
    mujoco.mj_forward(model, data)


    # --------------------------------------------------
    # RENDER CAMERA IMAGE
    # --------------------------------------------------

    renderer.update_scene(
        data,
        camera=camera_id
    )

    image = renderer.render()


    # --------------------------------------------------
    # RUN EXISTING COLOR BLOB TRACKER
    # --------------------------------------------------

    cx, cy, radius, found = tracker.find_target(image)


    # --------------------------------------------------
    # PRINT TRACKING RESULT
    # --------------------------------------------------

    if t - last_print_time >= 1.0:

        print(
            f"Time: {t:6.2f} s | "
            f"Target Y: {target_y:6.2f} m | "
            f"Found: {found} | "
            f"CX: {cx:7.2f} | "
            f"CY: {cy:7.2f} | "
            f"Radius: {radius:6.2f}"
        )

        last_print_time = t


    # --------------------------------------------------
    # SAVE IMAGE
    # --------------------------------------------------

    if t >= 2.0 and not saved_image:

        Image.fromarray(image).save(
            "tracker_camera_image.png"
        )

        print()
        print("==============================================")
        print(" TRACKER CAMERA IMAGE SAVED")
        print(" File: tracker_camera_image.png")
        print("==============================================")
        print()

        saved_image = True


    # --------------------------------------------------
    # STEP SIMULATION
    # --------------------------------------------------

    mujoco.mj_step(model, data)


# --------------------------------------------------
# CLEANUP
# --------------------------------------------------

renderer.close()

print("Tracker camera test finished.")