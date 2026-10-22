"""
target.py

Target manager module connecting continuous time-dependent trajectory
generator to MuJoCo body positions.
"""

import mujoco
import numpy as np
from src.trajectory import BaseTrajectory, get_trajectory

class TargetManager:
    """Manages moving target body position in MuJoCo based on simulation time."""

    def __init__(self, model: mujoco.MjModel, target_name: str = "target", trajectory: BaseTrajectory = None):
        self.model = model
        self.target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_name)
        if self.target_body_id < 0:
            raise ValueError(f"Target body '{target_name}' not found in MuJoCo model.")
            
        self.trajectory = trajectory if trajectory is not None else get_trajectory("circle")

    def set_trajectory(self, trajectory: BaseTrajectory):
        """Update active trajectory."""
        self.trajectory = trajectory

    def update(self, data: mujoco.MjData) -> np.ndarray:
        """
        Updates the target position in the MuJoCo model at current simulation time data.time.
        Returns the current target world position [x, y, z].
        """
        t = data.time
        pos = self.trajectory.get_position(t)
        self.model.body_pos[self.target_body_id] = pos
        mujoco.mj_forward(self.model, data)
        return pos.copy()

    def get_velocity(self, data: mujoco.MjData) -> np.ndarray:
        """Returns the analytical velocity [vx, vy, vz] of the target at current simulation time."""
        return self.trajectory.get_velocity(data.time)
