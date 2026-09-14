import mujoco
import numpy as np
from PIL import Image


# ============================================================
# LOAD MUJOCO MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path(
    "models/uav_target.xml"
)

data = mujoco.MjData(model)


# ============================================================
# GET OBJECT IDs
# ============================================================

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

uav_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "uav"
)


print("Camera ID:", camera_id)
print("Target body ID:", target_body)
print("UAV body ID:", uav_body)


# ============================================================
# IMAGE SIZE
# ============================================================

WIDTH = 320
HEIGHT = 240


# ============================================================
# CREATE RENDERER
# ============================================================

renderer = mujoco.Renderer(
    model,
    height=HEIGHT,
    width=WIDTH
)


# ============================================================
# INITIAL UAV POSITION
# ============================================================

uav_position = np.array([
    0.0,
    0.0,
    2.0
])


# ============================================================
# SIMULATION SETTINGS
# ============================================================

saved_image = False

last_print_time = -1.0


# ============================================================
# SIMULATION LOOP
# ============================================================

for step in range(5000):

    # Current simulation time

    t = data.time


    # ========================================================
    # KEEP UAV FIXED
    # ========================================================

    data.qpos[0:3] = uav_position

    # Keep UAV level
    #
    # Quaternion:
    # [w, x, y, z]
    #

    data.qpos[3:7] = np.array([
        1.0,
        0.0,
        0.0,
        0.0
    ])

    # No UAV velocity

    data.qvel[:] = 0.0


    # ========================================================
    # MOVE TARGET
    # ========================================================

    target_x = 2.0 + 0.5 * np.sin(0.5 * t)

    target_y = 0.0

    target_z = 2.5


    model.body_pos[target_body] = np.array([
        target_x,
        target_y,
        target_z
    ])


    # ========================================================
    # UPDATE MUJOCO
    # ========================================================

    mujoco.mj_forward(
        model,
        data
    )


    # ========================================================
    # RENDER CAMERA IMAGE
    # ========================================================

    renderer.update_scene(
        data,
        camera=camera_id
    )

    image = renderer.render()


    # ========================================================
    # PRINT INFORMATION EVERY SECOND
    # ========================================================

    if t - last_print_time >= 1.0:

        print(
            f"Time: {t:6.2f} s | "
            f"UAV: "
            f"({data.qpos[0]:5.2f}, "
            f"{data.qpos[1]:5.2f}, "
            f"{data.qpos[2]:5.2f}) | "
            f"Target X: {target_x:6.2f} m | "
            f"Image: {image.shape}"
        )

        last_print_time = t


    # ========================================================
    # SAVE IMAGE AFTER 2 SECONDS
    # ========================================================

    if t >= 2.0 and not saved_image:

        Image.fromarray(image).save(
            "camera_image.png"
        )

        print()
        print("==============================================")
        print(" CAMERA IMAGE SAVED SUCCESSFULLY")
        print(" File: camera_image.png")
        print("==============================================")
        print()

        saved_image = True


    # ========================================================
    # ADVANCE SIMULATION
    # ========================================================

    mujoco.mj_step(
        model,
        data
    )


# ============================================================
# CLOSE RENDERER
# ============================================================

renderer.close()


print("Simulation finished.")