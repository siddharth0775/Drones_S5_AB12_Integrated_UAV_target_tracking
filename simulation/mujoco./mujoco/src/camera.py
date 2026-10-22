"""
camera.py

MuJoCo camera manager for image acquisition, rendering, and intrinsic matrix computation.
"""

import mujoco
import numpy as np

class CameraManager:
    """Handles offscreen rendering and camera calibration/extrinsics from MuJoCo."""

    def __init__(self, model: mujoco.MjModel, camera_name: str = "uav_camera", width: int = 320, height: int = 240):
        self.model = model
        self.camera_name = camera_name
        self.width = width
        self.height = height

        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self.camera_id < 0:
            raise ValueError(f"Camera '{camera_name}' not found in MuJoCo model.")

        self.renderer = mujoco.Renderer(model, height=height, width=width)
        self.fovy_deg = float(model.cam_fovy[self.camera_id]) if self.camera_id >= 0 else 70.0
        self.K = self._compute_intrinsic_matrix()

    def _compute_intrinsic_matrix(self) -> np.ndarray:
        """Computes 3x3 camera intrinsic matrix K from FOVy and image dimensions."""
        fovy_rad = np.radians(self.fovy_deg)
        fy = (self.height / 2.0) / np.tan(fovy_rad / 2.0)
        fx = fy  # Assuming square pixels
        cx = self.width / 2.0
        cy = self.height / 2.0
        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    def capture_frame(self, data: mujoco.MjData) -> np.ndarray:
        """
        Renders live offscreen RGB frame from the UAV camera at current simulation step.
        Returns uint8 RGB numpy array of shape (height, width, 3).
        """
        self.renderer.update_scene(data, camera=self.camera_id)
        return self.renderer.render()

    def get_camera_pose(self, data: mujoco.MjData):
        """
        Returns absolute camera world position [x, y, z] and 3x3 orientation matrix R_cam_world.
        """
        pos = data.cam_xpos[self.camera_id].copy()
        rot_mat = data.cam_xmat[self.camera_id].reshape(3, 3).copy()
        return pos, rot_mat

    def close(self):
        """Releases renderer resources."""
        if hasattr(self, 'renderer') and self.renderer is not None:
            self.renderer.close()
