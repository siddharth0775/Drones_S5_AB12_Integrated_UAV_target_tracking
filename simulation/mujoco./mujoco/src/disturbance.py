"""
disturbance.py

Physical disturbance generator for UAV tracking evaluation.
Applies realistic force perturbations to the drone body to test
tracking robustness. Does NOT modify target, controller, or any other module.

Disturbance Types
-----------------
  none       - No disturbance (baseline)
  wind       - Constant directional wind force
  gust       - Random impulse bursts at intervals
  turbulence - Continuous band-limited random force noise
  all        - wind + gust + turbulence combined
"""

import numpy as np
import mujoco


class DisturbanceGenerator:
    """
    Applies physical force disturbances to the UAV body during simulation.
    Call apply() once per simulation step after controller forces are set.
    """

    def __init__(
        self,
        mode: str = "turbulence",
        wind_force: tuple = (0.3, 0.1, 0.0),   # Constant wind N (x,y,z world)
        gust_magnitude: float = 1.5,             # Peak gust force N
        gust_interval: float = 4.0,              # Avg seconds between gusts
        gust_duration: float = 0.4,              # Gust duration seconds
        turbulence_std: float = 0.25,            # Turbulence force std-dev N
        turbulence_bandwidth: float = 0.15,      # Low-pass filter coefficient
        seed: int = 0,
    ):
        self.mode = mode.lower().strip()
        self.wind = np.array(wind_force, dtype=np.float64)
        self.gust_magnitude = float(gust_magnitude)
        self.gust_interval = float(gust_interval)
        self.gust_duration = float(gust_duration)
        self.turb_std = float(turbulence_std)
        self.turb_bw = float(turbulence_bandwidth)

        self._rng = np.random.default_rng(seed)
        self._turb_force = np.zeros(3)          # Running turbulence filter state
        self._next_gust_t = self._rng.exponential(gust_interval)
        self._gust_end_t = -1.0
        self._gust_dir = np.zeros(3)

    def apply(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        uav_body_id: int,
    ):
        """
        Computes and applies disturbance forces to the UAV body for the current timestep.
        Uses mj_applyFT to inject forces at the UAV center of mass.
        """
        if self.mode == "none":
            return

        t = data.time
        force = np.zeros(3)

        # ---- Wind: constant directional push ----
        if self.mode in ("wind", "all"):
            force += self.wind

        # ---- Gust: scheduled random impulse burst ----
        if self.mode in ("gust", "all"):
            if t >= self._next_gust_t and t < self._gust_end_t:
                force += self._gust_dir
            elif t >= self._next_gust_t and t >= self._gust_end_t:
                # Start new gust
                self._gust_dir = self._rng.standard_normal(3)
                self._gust_dir /= (np.linalg.norm(self._gust_dir) + 1e-9)
                self._gust_dir *= self.gust_magnitude
                self._gust_end_t = t + self.gust_duration
                self._next_gust_t = t + self.gust_duration + self._rng.exponential(self.gust_interval)
                force += self._gust_dir

        # ---- Turbulence: low-pass filtered random noise ----
        if self.mode in ("turbulence", "all"):
            noise = self._rng.standard_normal(3) * self.turb_std
            self._turb_force = (
                self.turb_bw * noise + (1.0 - self.turb_bw) * self._turb_force
            )
            force += self._turb_force

        if np.any(force != 0):
            com = data.xpos[uav_body_id].copy()
            mujoco.mj_applyFT(
                model, data,
                force, np.zeros(3),
                com, uav_body_id,
                data.qfrc_applied
            )

    def describe(self) -> str:
        """Returns a human-readable description of the active disturbance."""
        if self.mode == "none":
            return "No disturbance"
        if self.mode == "wind":
            return f"Constant wind {self.wind} N"
        if self.mode == "gust":
            return f"Random gusts ±{self.gust_magnitude:.1f}N every ~{self.gust_interval:.1f}s"
        if self.mode == "turbulence":
            return f"Turbulence σ={self.turb_std:.2f}N"
        if self.mode == "all":
            return f"Wind+Gust+Turbulence (wind={self.wind}, gust±{self.gust_magnitude:.1f}N, turb σ={self.turb_std:.2f}N)"
        return self.mode
