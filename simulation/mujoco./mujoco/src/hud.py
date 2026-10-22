"""
hud.py

OpenCV-based camera HUD display module for MuJoCo UAV Tracking Simulation.

Mirrors the drone camera display window in PyBullet's track_demo.py, adding:
  - Live drone-eye-view feed
  - Crosshair
  - Target detection circle and tracking line
  - Tracking status overlay (TRACKING / SEARCHING)
  - Controller command readout (vx, vy, vz, yaw_rate)
  - Pixel error readout (ex, ey)
  - Keyboard control hint bar

Keyboard Controls (when window is in focus):
  W / S  : move target forward / backward  (+X / -X)
  A / D  : move target left / right        (-Y / +Y)
  R / F  : move target up / down           (+Z / -Z)
  ESC    : quit simulation
"""

import cv2
import numpy as np


class DroneHUD:
    """
    Renders a live OpenCV drone-camera HUD window, matching PyBullet track_demo.py's camera display.
    """

    WINDOW_NAME = "MuJoCo Drone Camera"

    def __init__(self, width: int = 320, height: int = 240):
        self.width = width
        self.height = height
        self._window_open = False

    def _ensure_window(self):
        if not self._window_open:
            cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.WINDOW_NAME, self.width * 2, self.height * 2)
            self._window_open = True

    def render(
        self,
        rgb_frame: np.ndarray,
        tracker_info: dict,
        ctrl_out: dict,
        pixel_error_x: float = 0.0,
        pixel_error_y: float = 0.0,
        sim_time: float = 0.0,
        keyboard_mode: bool = False,
    ) -> int:
        """
        Draws the HUD overlay on the given RGB frame and shows it in an OpenCV window.

        Returns the pressed key code from cv2.waitKey(1) (0-255), or -1 if none.
        """
        self._ensure_window()
        hud = rgb_frame.copy()

        # ----------------------------------------------------------------
        # Crosshair at image center
        # ----------------------------------------------------------------
        cx_img = self.width // 2
        cy_img = self.height // 2
        cv2.drawMarker(hud, (cx_img, cy_img), (255, 255, 255),
                       cv2.MARKER_CROSS, 15, 1)
        cv2.putText(hud, "TARGET: MAGENTA BOX", (10, self.height - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)

        found = tracker_info.get("found", False)
        state = tracker_info.get("state", "SEARCHING")

        if found:
            tx = int(tracker_info.get("cx", cx_img))
            ty = int(tracker_info.get("cy", cy_img))
            trad = int(max(tracker_info.get("radius_px", 10), 2))

            # Detection circle (green)
            cv2.circle(hud, (tx, ty), trad, (0, 255, 0), 2)

            # Error line from crosshair to target
            cv2.line(hud, (cx_img, cy_img), (tx, ty), (255, 255, 0), 1)

            # Tracking status label
            cv2.putText(hud, "TRACKING", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Normalized pixel errors
            norm_ex = pixel_error_x / (self.width / 2.0)
            norm_ey = pixel_error_y / (self.height / 2.0)
            cv2.putText(hud, f"ex: {norm_ex:+.2f}", (self.width - 110, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
            cv2.putText(hud, f"ey: {norm_ey:+.2f}", (self.width - 110, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        else:
            # Searching / Lost status label
            color = (0, 165, 255) if state == "SEARCHING" else (0, 0, 255)
            cv2.putText(hud, state, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ----------------------------------------------------------------
        # Controller command readout (left column)
        # ----------------------------------------------------------------
        vx  = ctrl_out.get("des_vx",      0.0)
        vy  = ctrl_out.get("des_vy",      0.0)
        vz  = ctrl_out.get("des_vz",      0.0)
        yaw = ctrl_out.get("des_yaw_rate", 0.0)

        cv2.putText(hud, f"vx: {vx:+.2f}", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        cv2.putText(hud, f"vy: {vy:+.2f}", (10, 63),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        cv2.putText(hud, f"vz: {vz:+.2f}", (10, 81),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        cv2.putText(hud, f"yaw: {yaw:+.2f}", (10, 99),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # Simulation time (top-right)
        cv2.putText(hud, f"T:{sim_time:.1f}s", (self.width - 75, self.height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

        # ----------------------------------------------------------------
        # Keyboard hint bar (bottom)
        # ----------------------------------------------------------------
        if keyboard_mode:
            hint = "WASD/RF: move target | ESC: quit"
        else:
            hint = "ESC: quit"
        cv2.putText(hud, hint, (10, self.height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

        # ----------------------------------------------------------------
        # Show window
        # ----------------------------------------------------------------
        cv2.imshow(self.WINDOW_NAME, cv2.cvtColor(hud, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 0xFF
        return key

    def close(self):
        """Closes the HUD window."""
        if self._window_open:
            cv2.destroyWindow(self.WINDOW_NAME)
            self._window_open = False


def parse_keyboard_target_delta(key: int, move_speed: float = 1.5, dt: float = 0.002) -> np.ndarray:
    """
    Translates an OpenCV key code into a target displacement delta [dx, dy, dz].

    Controls (matching PyBullet track_demo.py):
        W / w  : +X  (forward)
        S / s  : -X  (backward)
        A / a  : -Y  (left)
        D / d  : +Y  (right)
        R / r  : +Z  (up)
        F / f  : -Z  (down)
    """
    delta = np.zeros(3, dtype=np.float64)
    step = move_speed * dt

    if key in (ord('w'), ord('W')):
        delta[0] += step
    elif key in (ord('s'), ord('S')):
        delta[0] -= step
    elif key in (ord('a'), ord('A')):
        delta[1] -= step
    elif key in (ord('d'), ord('D')):
        delta[1] += step
    elif key in (ord('r'), ord('R')):
        delta[2] += step
    elif key in (ord('f'), ord('F')):
        delta[2] -= step

    return delta
