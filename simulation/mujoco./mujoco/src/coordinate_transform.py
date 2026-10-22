"""
coordinate_transform.py

Complete coordinate pipeline:
IMAGE PIXEL -> CAMERA FRAME -> UAV BODY FRAME -> WORLD FRAME
Includes 3D target localization and physical depth estimation from bounding box / radius.
"""

from typing import Tuple, Optional
import numpy as np
import mujoco

class CoordinateTransformer:
    """Transforms pixel coordinates to 3D camera, body, and world coordinates."""

    def __init__(self, target_radius_real: float = 0.30):
        self.target_radius_real = target_radius_real

    def estimate_depth(self, radius_px: float, fy: float) -> float:
        """
        Estimates distance/depth (meters) from optical center to target sphere center
        using pinhole geometry and known physical target sphere radius.
        r_px = fy * R_real / depth => depth = fy * R_real / r_px
        """
        if radius_px <= 1e-3:
            return 0.0
        return float((fy * self.target_radius_real) / radius_px)

    def pixel_to_camera_frame(self, cx: float, cy: float, radius_px: float, K: np.ndarray) -> np.ndarray:
        """
        Backprojects pixel center (cx, cy) to 3D camera coordinate system [X_cam, Y_cam, Z_cam].
        In camera optical frame: Z_cam = depth forward, X_cam = right, Y_cam = down.
        """
        fx = K[0, 0]
        fy = K[1, 1]
        c_x = K[0, 2]
        c_y = K[1, 2]

        depth = self.estimate_depth(radius_px, fy)
        if depth <= 0.0:
            return np.zeros(3, dtype=np.float64)

        # Ray direction in camera coordinate frame
        x_cam = (cx - c_x) * depth / fx
        y_cam = (cy - c_y) * depth / fy
        z_cam = depth

        return np.array([x_cam, y_cam, z_cam], dtype=np.float64)

    def camera_to_world_frame(
        self,
        p_cam: np.ndarray,
        cam_pos_world: np.ndarray,
        R_cam_world: np.ndarray
    ) -> np.ndarray:
        """
        Transforms 3D position from camera frame to world frame.
        p_world = cam_pos_world + R_cam_world @ p_cam_mujoco
        """
        # Note: MuJoCo camera coordinate frame convention:
        # Camera looks towards -Z_cam, +Y_cam is up, +X_cam is right in camera space.
        # Standard conversion for OpenCV camera (optical axis +Z): [x_cam, -y_cam, -z_cam]
        p_cam_mujoco = np.array([p_cam[0], -p_cam[1], -p_cam[2]], dtype=np.float64)
        return cam_pos_world + R_cam_world @ p_cam_mujoco

    def world_to_uav_body_frame(
        self,
        p_world: np.ndarray,
        uav_pos_world: np.ndarray,
        R_uav_world: np.ndarray
    ) -> np.ndarray:
        """
        Transforms 3D position from world frame to UAV local body frame.
        p_body = R_uav_world^T @ (p_world - uav_pos_world)
        """
        return R_uav_world.T @ (p_world - uav_pos_world)

    def localize_target_3d(
        self,
        cx: float,
        cy: float,
        radius_px: float,
        K: np.ndarray,
        data: mujoco.MjData,
        camera_id: int,
        uav_body_id: int
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Full end-to-end 3D localization pipeline:
        Pixel -> Camera 3D -> World 3D -> UAV Body Relative 3D.
        
        Returns:
            rel_pos_body: np.ndarray [x_rel, y_rel, z_rel] in UAV body frame
            target_pos_world_est: np.ndarray [x, y, z] estimated world position
            estimated_depth: float distance in meters
        """
        # Step 1: Pixel -> 3D Camera Ray
        p_cam = self.pixel_to_camera_frame(cx, cy, radius_px, K)
        depth = p_cam[2]

        # Step 2: Get camera pose in world frame
        cam_pos_world = data.cam_xpos[camera_id].copy()
        R_cam_world = data.cam_xmat[camera_id].reshape(3, 3).copy()

        # Step 3: Camera -> World
        target_pos_world_est = self.camera_to_world_frame(p_cam, cam_pos_world, R_cam_world)

        # Step 4: World -> UAV Body Frame
        uav_pos_world = data.xpos[uav_body_id].copy()
        quat_uav = data.qpos[3:7]
        rot_arr = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_arr, quat_uav)
        R_uav_world = rot_arr.reshape(3, 3)

        rel_pos_body = self.world_to_uav_body_frame(target_pos_world_est, uav_pos_world, R_uav_world)

        return rel_pos_body, target_pos_world_est, depth
