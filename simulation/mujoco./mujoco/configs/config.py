"""
config.py

Centralized configuration for MuJoCo UAV Target Tracking System.
All system gains, simulation options, trajectory parameters, camera intrinsics,
and safety limits are defined here.
"""

import numpy as np

class SimConfig:
    # Simulation Parameters
    TIMESTEP = 0.002           # Physics step time in seconds (500 Hz)
    DURATION = 30.0            # Default simulation time in seconds
    MODEL_PATH = "models/uav_target.xml"
    
    # Camera Parameters
    RENDER_WIDTH = 320
    RENDER_HEIGHT = 240
    CAMERA_NAME = "uav_camera"
    FOVY_DEG = 70.0            # Camera vertical FOV in degrees
    
    # Target Parameters
    TARGET_NAME = "target"
    TARGET_RADIUS_REAL = 0.30  # Real-world target sphere radius in meters
    DEFAULT_TRAJECTORY = "circle"
    # All available trajectory types (10 total, including 4 new MuJoCo-native)
    AVAILABLE_TRAJECTORIES = [
        "stationary", "line", "circle", "sine", "figure8", "waypoint",  # Original (matches PyBullet)
        "helix", "lissajous", "zigzag", "wander",                          # New MuJoCo-native
    ]

    # Relative Following Desired State
    DESIRED_FOLLOWING_DISTANCE = 2.0 # Desired distance in front of UAV (meters)
    DESIRED_RELATIVE_HEIGHT = 0.0     # Target height relative to UAV (meters)
    DESIRED_PIXEL_RADIUS = 30.0       # Target image radius benchmark (pixels)

    # Controller Gains (Centralized PID tuning)
    # Forward Translation (Vx control -> Pitch mapping)
    KP_VX = 1.2
    KD_VX = 0.3
    MAX_VX = 2.5               # Max forward speed (m/s)

    # Lateral Translation (Vy control -> Roll mapping)
    KP_VY = 1.2
    KD_VY = 0.3
    MAX_VY = 2.0               # Max lateral speed (m/s)

    # Altitude Control (Vz / Z position)
    KP_Z = 15.0
    KD_Z = 6.0
    MAX_VZ = 2.0               # Max vertical speed (m/s)

    # Attitude Stabilization (Roll & Pitch PIDs)
    KP_ROLL = 2.5
    KD_ROLL = 0.6
    KP_PITCH = 2.5
    KD_PITCH = 0.6

    # Yaw Control (Heading alignment to target)
    KP_YAW = 1.8
    KD_YAW = 0.4
    MAX_YAW_RATE = 1.5         # Max yaw rate (rad/s)
    YAW_TORQUE_SCALE = 0.15    # Max yaw torque scaling

    # Quadrotor Physical Properties
    MASS = 1.0                 # Quadrotor mass in kg
    GRAVITY = 9.81             # Gravity acceleration m/s^2

    # Target Loss & Safety Limits
    TARGET_LOST_TIMEOUT = 2.0  # Time in seconds before full search mode
    MAX_ALTITUDE = 10.0        # Upper flight altitude boundary (meters)
    MIN_ALTITUDE = 0.3         # Lower flight altitude boundary (meters)
    BOUNDS_X = (-15.0, 15.0)   # Workspace boundary X
    BOUNDS_Y = (-15.0, 15.0)   # Workspace boundary Y

    # HUD & Display (matches PyBullet track_demo.py camera window)
    SHOW_HUD = True            # Show OpenCV drone-camera window by default
    HUD_SCALE = 2              # Scale factor for HUD window (2 = 640x480 from 320x240)

    # Keyboard Control (matches PyBullet WASD/RF target control)
    KEYBOARD_TARGET_SPEED = 1.5  # m/s manual target movement speed
