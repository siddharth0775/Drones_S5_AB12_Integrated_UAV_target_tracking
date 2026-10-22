"""
trajectory.py

Clean, time-dependent target trajectory generation system.
Supports continuous, smooth trajectories for simulation evaluation.

Trajectory Types
----------------
Original (also present in PyBullet):
  stationary  - Fixed position
  line        - Straight-line constant velocity
  circle      - 2D circle with optional Z oscillation (mirrors PyBullet 'circle')
  figure8     - Lemniscate figure-8 path (mirrors PyBullet 'figure8')
  waypoint    - Smooth multi-waypoint path (mirrors PyBullet 'waypoints')
  wander      - Random-walk goal-seeking (mirrors PyBullet 'wander')

MuJoCo-native (new trajectories extending PyBullet coverage):
  sine        - Sinusoidal XY oscillation with X drift
  helix       - 3D ascending/descending corkscrew
  lissajous   - 3D Lissajous curve (independent-frequency oscillations)
  zigzag      - Sharp lateral zig-zag evasive maneuver (mirrors PyBullet WASD manual)
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseTrajectory(ABC):
    """Abstract base class for target trajectories."""

    @abstractmethod
    def get_position(self, t: float) -> np.ndarray:
        """Returns target [x, y, z] position at time t."""
        pass

    @abstractmethod
    def get_velocity(self, t: float) -> np.ndarray:
        """Returns target [vx, vy, vz] velocity at time t."""
        pass


# ----------------------------------------------------------------
# Original Trajectories
# ----------------------------------------------------------------

class StationaryTrajectory(BaseTrajectory):
    """Target remains stationary at a fixed position."""

    def __init__(self, position=(3.0, 0.0, 2.5)):
        self.position = np.array(position, dtype=np.float64)

    def get_position(self, t: float) -> np.ndarray:
        return self.position.copy()

    def get_velocity(self, t: float) -> np.ndarray:
        return np.zeros(3, dtype=np.float64)


class StraightLineTrajectory(BaseTrajectory):
    """Target moves in a straight line with constant velocity."""

    def __init__(self, start_pos=(3.0, 0.0, 2.5), velocity=(0.4, 0.0, 0.0)):
        self.start_pos = np.array(start_pos, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)

    def get_position(self, t: float) -> np.ndarray:
        return self.start_pos + self.velocity * t

    def get_velocity(self, t: float) -> np.ndarray:
        return self.velocity.copy()


class CircularTrajectory(BaseTrajectory):
    """Target moves in a 3D circle with optional Z oscillation. Mirrors PyBullet 'circle'."""

    def __init__(self, center=(3.0, 0.0, 2.5), radius=1.8, omega=0.35, z_amp=0.2, z_freq=0.3):
        self.center = np.array(center, dtype=np.float64)
        self.radius = float(radius)
        self.omega = float(omega)
        self.z_amp = float(z_amp)
        self.z_freq = float(z_freq)

    def get_position(self, t: float) -> np.ndarray:
        x = self.center[0] + self.radius * np.cos(self.omega * t)
        y = self.center[1] + self.radius * np.sin(self.omega * t)
        z = self.center[2] + self.z_amp * np.sin(self.z_freq * t)
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        vx = -self.radius * self.omega * np.sin(self.omega * t)
        vy = self.radius * self.omega * np.cos(self.omega * t)
        vz = self.z_amp * self.z_freq * np.cos(self.z_freq * t)
        return np.array([vx, vy, vz], dtype=np.float64)


class SinusoidalTrajectory(BaseTrajectory):
    """Target drifts along X while oscillating in Y and Z with sine waves."""

    def __init__(self, base_pos=(3.0, 0.0, 2.5), speed_x=0.0,
                 amp_y=1.5, freq_y=0.4, amp_z=0.3, freq_z=0.5):
        self.base_pos = np.array(base_pos, dtype=np.float64)
        self.speed_x = float(speed_x)
        self.amp_y = float(amp_y)
        self.freq_y = float(freq_y)
        self.amp_z = float(amp_z)
        self.freq_z = float(freq_z)

    def get_position(self, t: float) -> np.ndarray:
        x = self.base_pos[0] + self.speed_x * t
        y = self.base_pos[1] + self.amp_y * np.sin(self.freq_y * t)
        z = self.base_pos[2] + self.amp_z * np.cos(self.freq_z * t)
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        vx = self.speed_x
        vy = self.amp_y * self.freq_y * np.cos(self.freq_y * t)
        vz = -self.amp_z * self.freq_z * np.sin(self.freq_z * t)
        return np.array([vx, vy, vz], dtype=np.float64)


class Figure8Trajectory(BaseTrajectory):
    """Target moves in a figure-8 (Bernoulli lemniscate). Mirrors PyBullet 'figure8'."""

    def __init__(self, center=(3.0, 0.0, 2.5), scale_x=2.0, scale_y=1.2, speed=0.35, z_amp=0.2):
        self.center = np.array(center, dtype=np.float64)
        self.scale_x = float(scale_x)
        self.scale_y = float(scale_y)
        self.speed = float(speed)
        self.z_amp = float(z_amp)

    def get_position(self, t: float) -> np.ndarray:
        st = np.sin(self.speed * t)
        ct = np.cos(self.speed * t)
        x = self.center[0] + self.scale_x * st
        y = self.center[1] + self.scale_y * st * ct
        z = self.center[2] + self.z_amp * np.sin(2 * self.speed * t)
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        ct = np.cos(self.speed * t)
        c2t = np.cos(2 * self.speed * t)
        vx = self.scale_x * self.speed * ct
        vy = self.scale_y * self.speed * c2t
        vz = 2 * self.z_amp * self.speed * np.cos(2 * self.speed * t)
        return np.array([vx, vy, vz], dtype=np.float64)


class WaypointTrajectory(BaseTrajectory):
    """Target smoothly transitions through a list of 3D waypoints. Mirrors PyBullet 'waypoints'."""

    def __init__(self, waypoints=None, speed=0.8):
        if waypoints is None:
            waypoints = [
                [3.0,  0.0, 2.5],
                [4.0,  1.5, 2.8],
                [4.5, -1.5, 2.3],
                [3.5,  0.0, 2.7],
                [3.0,  0.0, 2.5],
            ]
        self.waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        self.speed = float(speed)
        self.segment_vectors = []
        self.segment_lengths = []
        self.segment_times = []
        for i in range(len(self.waypoints) - 1):
            vec = self.waypoints[i + 1] - self.waypoints[i]
            length = max(np.linalg.norm(vec), 1e-6)
            self.segment_vectors.append(vec / length)
            self.segment_lengths.append(length)
            self.segment_times.append(length / self.speed)
        self.total_time = sum(self.segment_times)

    def get_position(self, t: float) -> np.ndarray:
        if self.total_time <= 0:
            return self.waypoints[0].copy()
        t_mod = t % self.total_time
        accum_t = 0.0
        for i, seg_t in enumerate(self.segment_times):
            if accum_t <= t_mod <= accum_t + seg_t:
                dt = t_mod - accum_t
                return self.waypoints[i] + self.segment_vectors[i] * (self.speed * dt)
            accum_t += seg_t
        return self.waypoints[-1].copy()

    def get_velocity(self, t: float) -> np.ndarray:
        if self.total_time <= 0:
            return np.zeros(3, dtype=np.float64)
        t_mod = t % self.total_time
        accum_t = 0.0
        for i, seg_t in enumerate(self.segment_times):
            if accum_t <= t_mod <= accum_t + seg_t:
                return self.segment_vectors[i] * self.speed
            accum_t += seg_t
        return np.zeros(3, dtype=np.float64)


# ----------------------------------------------------------------
# New Trajectories  –  extending PyBullet coverage in MuJoCo
# ----------------------------------------------------------------

class HelixTrajectory(BaseTrajectory):
    """
    3-D corkscrew helix: circular XY orbit + triangular altitude bounce.

    Inspired by combining PyBullet circle + manual R/F altitude keys into
    a single autonomous trajectory. Altitude bounces smoothly between
    z_min and z_max at the given climb_rate (m/s).
    """

    def __init__(self, center=(3.0, 0.0, 1.5), radius=1.5, omega=0.4,
                 climb_rate=0.08, z_max=4.0, z_min=1.5):
        self.center = np.array(center, dtype=np.float64)
        self.radius = float(radius)
        self.omega = float(omega)
        self.z_max = float(z_max)
        self.z_min = float(z_min)
        # Full up+down cycle period
        self.altitude_period = 2.0 * (z_max - z_min) / max(abs(climb_rate), 1e-6)

    def get_position(self, t: float) -> np.ndarray:
        x = self.center[0] + self.radius * np.cos(self.omega * t)
        y = self.center[1] + self.radius * np.sin(self.omega * t)
        z_range = self.z_max - self.z_min
        phase = (t % self.altitude_period) / self.altitude_period
        z_norm = 1.0 - abs(2.0 * phase - 1.0)   # triangular wave [0, 1]
        z = self.z_min + z_range * z_norm
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        vx = -self.radius * self.omega * np.sin(self.omega * t)
        vy = self.radius * self.omega * np.cos(self.omega * t)
        z_range = self.z_max - self.z_min
        phase = (t % self.altitude_period) / self.altitude_period
        vz = (z_range / self.altitude_period) * (2.0 if phase < 0.5 else -2.0)
        return np.array([vx, vy, vz], dtype=np.float64)


class LissajousTrajectory(BaseTrajectory):
    """
    3-D Lissajous curve — all three axes oscillate at independent frequencies.

    When freq_x:freq_y:freq_z are rational the path eventually closes, producing
    a compact repeating figure. Irrational ratios produce a quasi-random
    space-filling pattern — perfect for long-horizon tracking stress tests.
    """

    def __init__(self, center=(3.0, 0.0, 2.5),
                 amp_x=1.5, amp_y=1.2, amp_z=0.6,
                 freq_x=0.3, freq_y=0.5, freq_z=0.2,
                 phase_y=np.pi / 4, phase_z=np.pi / 2):
        self.center = np.array(center, dtype=np.float64)
        self.amp_x = float(amp_x)
        self.amp_y = float(amp_y)
        self.amp_z = float(amp_z)
        self.freq_x = float(freq_x)
        self.freq_y = float(freq_y)
        self.freq_z = float(freq_z)
        self.phase_y = float(phase_y)
        self.phase_z = float(phase_z)

    def get_position(self, t: float) -> np.ndarray:
        x = self.center[0] + self.amp_x * np.sin(self.freq_x * t)
        y = self.center[1] + self.amp_y * np.sin(self.freq_y * t + self.phase_y)
        z = self.center[2] + self.amp_z * np.sin(self.freq_z * t + self.phase_z)
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        vx = self.amp_x * self.freq_x * np.cos(self.freq_x * t)
        vy = self.amp_y * self.freq_y * np.cos(self.freq_y * t + self.phase_y)
        vz = self.amp_z * self.freq_z * np.cos(self.freq_z * t + self.phase_z)
        return np.array([vx, vy, vz], dtype=np.float64)


class ZigZagTrajectory(BaseTrajectory):
    """
    Sharp lateral zig-zag evasive trajectory.

    Directly mirrors the movement produced when a PyBullet operator
    rapidly alternates A/D (left/right) target controls. The target
    flips between +lateral_amp and -lateral_amp every switch_time
    seconds while drifting slowly forward.
    """

    def __init__(self, center=(3.0, 0.0, 2.5), lateral_amp=2.0, forward_speed=0.3,
                 switch_time=2.5, z_amp=0.25, z_freq=0.4):
        self.center = np.array(center, dtype=np.float64)
        self.lateral_amp = float(lateral_amp)
        self.forward_speed = float(forward_speed)
        self.switch_time = float(switch_time)
        self.z_amp = float(z_amp)
        self.z_freq = float(z_freq)

    def get_position(self, t: float) -> np.ndarray:
        # X: slow forward drift, wraps to avoid unbounded growth
        x = self.center[0] + self.forward_speed * (t % 20.0)
        # Y: square-wave lateral flip
        cycle = t % (2 * self.switch_time)
        y_dir = 1.0 if cycle < self.switch_time else -1.0
        y = self.center[1] + y_dir * self.lateral_amp
        # Z: sinusoidal altitude variation
        z = self.center[2] + self.z_amp * np.sin(self.z_freq * t)
        return np.array([x, y, z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        vx = self.forward_speed
        vy = 0.0  # Instantaneous switch — velocity ≈ 0 between transitions
        vz = self.z_amp * self.z_freq * np.cos(self.z_freq * t)
        return np.array([vx, vy, vz], dtype=np.float64)


class WanderTrajectory(BaseTrajectory):
    """
    Random-walk 'wander' trajectory.

    Target moves toward a randomly chosen goal within wander_radius
    of its home center, then picks a new goal on arrival. Uses a fixed
    seed for reproducible evaluation runs.

    Directly mirrors PyBullet moving_target.py 'wander' path_type.
    """

    def __init__(self, center=(3.0, 0.0, 2.5), wander_radius=2.0, speed=0.5,
                 seed=42, z_amp=0.3, z_freq=0.25):
        center = tuple(center)
        self.center_xy = np.array(center[:2], dtype=np.float64)
        self.base_z = float(center[2]) if len(center) > 2 else 2.5
        self.wander_radius = float(wander_radius)
        self.speed = float(speed)
        self.z_amp = float(z_amp)
        self.z_freq = float(z_freq)
        self._rng = np.random.default_rng(seed)
        self._pos = self.center_xy.copy()
        self._goal = self._pick_goal()
        self._last_t = 0.0

    def _pick_goal(self) -> np.ndarray:
        angle = self._rng.uniform(0, 2 * np.pi)
        r = self._rng.uniform(0, self.wander_radius)
        return self.center_xy + np.array([r * np.cos(angle), r * np.sin(angle)])

    def _step(self, dt: float):
        direction = self._goal - self._pos
        dist = np.linalg.norm(direction)
        step = self.speed * dt
        if dist < step + 0.05:
            self._pos = self._goal.copy()
            self._goal = self._pick_goal()
        else:
            self._pos += (direction / dist) * step

    def get_position(self, t: float) -> np.ndarray:
        dt = t - self._last_t
        if dt > 0:
            n_steps = max(1, int(dt / 0.02))
            sub_dt = dt / n_steps
            for _ in range(n_steps):
                self._step(sub_dt)
        self._last_t = t
        z = self.base_z + self.z_amp * np.sin(self.z_freq * t)
        return np.array([self._pos[0], self._pos[1], z], dtype=np.float64)

    def get_velocity(self, t: float) -> np.ndarray:
        direction = self._goal - self._pos
        dist = np.linalg.norm(direction)
        vxy = (direction / dist) * self.speed if dist > 1e-6 else np.zeros(2)
        vz = self.z_amp * self.z_freq * np.cos(self.z_freq * t)
        return np.array([vxy[0], vxy[1], vz], dtype=np.float64)


# ----------------------------------------------------------------
# Factory
# ----------------------------------------------------------------

def get_trajectory(trajectory_type: str, **kwargs) -> BaseTrajectory:
    """Factory function to instantiate trajectories by name."""
    type_map = {
        # Original trajectories
        "stationary": StationaryTrajectory,
        "line":       StraightLineTrajectory,
        "circle":     CircularTrajectory,
        "sine":       SinusoidalTrajectory,
        "figure8":    Figure8Trajectory,
        "waypoint":   WaypointTrajectory,
        # New MuJoCo-native trajectories (extending PyBullet coverage)
        "helix":      HelixTrajectory,
        "lissajous":  LissajousTrajectory,
        "zigzag":     ZigZagTrajectory,
        "wander":     WanderTrajectory,
    }
    key = trajectory_type.lower().strip()
    if key not in type_map:
        raise ValueError(
            f"Unknown trajectory type '{trajectory_type}'. "
            f"Available: {list(type_map.keys())}"
        )
    return type_map[key](**kwargs)
