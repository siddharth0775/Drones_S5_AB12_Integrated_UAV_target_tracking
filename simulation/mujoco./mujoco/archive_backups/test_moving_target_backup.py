import mujoco
import mujoco.viewer
import numpy as np

# Load target model
model = mujoco.MjModel.from_xml_path(
    "models/target_test.xml"
)

data = mujoco.MjData(model)

# Find target body
target_body = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "target"
)

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Current simulation time
        t = data.time

        # Move target left and right
        x = 2.0 * np.sin(0.5 * t)

        # Keep target at fixed Y and Z
        y = 0.0
        z = 1.5

        # Target body position
        data.xpos[target_body]

        # Since target has no joint, change its XML-defined
        # position through the body position in the model.
        model.body_pos[target_body] = [x, y, z]

        # Update MuJoCo kinematics
        mujoco.mj_forward(model, data)

        # Advance simulation
        mujoco.mj_step(model, data)

        viewer.sync()