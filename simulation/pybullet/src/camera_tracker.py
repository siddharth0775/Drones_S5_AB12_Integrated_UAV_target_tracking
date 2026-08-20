"""
camera_tracker.py

Vision + tracking controller for the PyBullet object-tracking drone.

ColorBlobTracker:
    Detects the green target using HSV thresholding.

PIDController:
    Reusable PID controller with:
        - integral limiting
        - derivative filtering
        - output limiting

TrackingController:
    Converts image-space target error into:
        vx       forward/backward velocity
        vy       lateral velocity
        vz       vertical velocity
        yaw_rate yaw correction

The controller intentionally avoids aggressive simultaneous
forward motion + yaw. The target is first brought toward the
image center, then the drone moves toward/away from it.
"""

import cv2
import numpy as np


class ColorBlobTracker:

    def __init__(
        self,
        lower_hsv=(35, 80, 80),
        upper_hsv=(85, 255, 255),
        min_area=15,
        smoothing=0.35,
    ):
        self.lower = np.array(lower_hsv, dtype=np.uint8)
        self.upper = np.array(upper_hsv, dtype=np.uint8)

        self.min_area = min_area
        self.smoothing = smoothing

        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None

    def reset(self):
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None

    def find_target(self, rgb_frame):

        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(
            hsv,
            self.lower,
            self.upper
        )

        # Remove isolated noise.
        kernel = np.ones((3, 3), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None, None, False

        # Ignore very small blobs.
        candidates = [
            c for c in contours
            if cv2.contourArea(c) >= self.min_area
        ]

        if not candidates:
            return None, None, None, False

        # Largest valid green object.
        target = max(
            candidates,
            key=cv2.contourArea
        )

        area = cv2.contourArea(target)

        (cx, cy), radius = cv2.minEnclosingCircle(target)

        if radius <= 1.0:
            return None, None, None, False

        # ---------------------------------------------------------
        # Smooth the detected target.
        # ---------------------------------------------------------

        if self.filtered_cx is None:

            self.filtered_cx = cx
            self.filtered_cy = cy
            self.filtered_radius = radius

        else:

            a = self.smoothing

            self.filtered_cx = (
                a * cx +
                (1.0 - a) * self.filtered_cx
            )

            self.filtered_cy = (
                a * cy +
                (1.0 - a) * self.filtered_cy
            )

            self.filtered_radius = (
                a * radius +
                (1.0 - a) * self.filtered_radius
            )

        return (
            float(self.filtered_cx),
            float(self.filtered_cy),
            float(self.filtered_radius),
            True
        )


class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_limit=None,
        integral_limit=None,
        derivative_filter=0.25,
    ):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.derivative_filter = derivative_filter

        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0

    def reset(self):

        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0

    def step(self, error, dt):

        if dt <= 0:
            return 0.0

        # Integral
        self._integral += error * dt

        if self.integral_limit is not None:

            self._integral = float(
                np.clip(
                    self._integral,
                    -self.integral_limit,
                    self.integral_limit
                )
            )

        # Raw derivative
        derivative = (
            error - self._prev_error
        ) / dt

        self._prev_error = error

        # Low-pass filter derivative
        a = self.derivative_filter

        self._filtered_derivative = (
            a * derivative
            + (1.0 - a) * self._filtered_derivative
        )

        output = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * self._filtered_derivative
        )

        if self.output_limit is not None:

            output = float(
                np.clip(
                    output,
                    -self.output_limit,
                    self.output_limit
                )
            )

        return output


class TrackingController:

    def __init__(self, target_radius_px=30):

        # ---------------------------------------------------------
        # Horizontal centering.
        #
        # Pixel error is normalized to approximately [-1, +1].
        #
        # +error = target is RIGHT
        # -error = target is LEFT
        # ---------------------------------------------------------

        self.yaw_pid = PIDController(
            kp=1.8,
            ki=0.0,
            kd=0.12,
            output_limit=1.5,
            integral_limit=0.3,
            derivative_filter=0.2,
        )

        # ---------------------------------------------------------
        # Distance controller.
        #
        # +error = target too small = too far
        # -error = target too large = too close
        # ---------------------------------------------------------

        self.dist_pid = PIDController(
            kp=0.035,
            ki=0.0,
            kd=0.008,
            output_limit=0.7,
            integral_limit=0.2,
            derivative_filter=0.2,
        )

        # ---------------------------------------------------------
        # Vertical controller.
        # ---------------------------------------------------------

        self.alt_pid = PIDController(
            kp=0.55,
            ki=0.0,
            kd=0.08,
            output_limit=0.5,
            integral_limit=0.2,
            derivative_filter=0.2,
        )

        self.target_radius_px = target_radius_px

        self.state = "SEARCHING"

        self.lost_frames = 0

        self.reacquire_grace_frames = 24

        self.last_seen_dir = 1.0

        self.last_seen_vx = 0.0

        # Deadband prevents constant tiny corrections.
        self.horizontal_deadband = 0.06
        self.vertical_deadband = 0.06

        # The target must be reasonably centered before
        # the drone is allowed to move aggressively forward.
        self.forward_alignment_limit = 0.18

    def reset(self):

        self.yaw_pid.reset()
        self.dist_pid.reset()
        self.alt_pid.reset()

        self.state = "SEARCHING"

        self.lost_frames = 0
        self.last_seen_dir = 1.0
        self.last_seen_vx = 0.0

    def update(
        self,
        cx,
        cy,
        radius_px,
        frame_w,
        frame_h,
        found,
        dt
    ):

        # =========================================================
        # TARGET LOST
        # =========================================================

        if not found:

            self.lost_frames += 1
            self.state = "SEARCHING"

            # Stop distance/altitude PID accumulation while
            # target is not visible.
            self.dist_pid.reset()
            self.alt_pid.reset()

            if self.lost_frames <= self.reacquire_grace_frames:

                return {
                    "vx": 0.0,
                    "vy": 0.0,
                    "vz": 0.0,
                    "yaw_rate":
                        0.6 * self.last_seen_dir,
                }

            return {
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw_rate": 0.25,
            }

        # =========================================================
        # TARGET FOUND
        # =========================================================

        self.state = "TRACKING"
        self.lost_frames = 0

        center_x = frame_w / 2.0
        center_y = frame_h / 2.0

        # ---------------------------------------------------------
        # Normalize image errors.
        # ---------------------------------------------------------

        error_x = (
            (cx - center_x)
            / center_x
        )

        error_y = (
            (center_y - cy)
            / center_y
        )

        # +error = target too far
        distance_error = (
            self.target_radius_px
            - radius_px
        )

        # ---------------------------------------------------------
        # Deadband
        # ---------------------------------------------------------

        if abs(error_x) < self.horizontal_deadband:
            error_x = 0.0

        if abs(error_y) < self.vertical_deadband:
            error_y = 0.0

        # ---------------------------------------------------------
        # YAW
        #
        # Target right -> positive yaw.
        # Target left  -> negative yaw.
        # ---------------------------------------------------------

        yaw_rate = -self.yaw_pid.step(
            error_x,
            dt
        )

        # ---------------------------------------------------------
        # ALTITUDE
        #
        # Target above -> positive vz.
        # Target below -> negative vz.
        # ---------------------------------------------------------

        vz = self.alt_pid.step(
            error_y,
            dt
        )

        # ---------------------------------------------------------
        # FORWARD/BACKWARD
        #
        # IMPORTANT:
        #
        # Don't rush forward while the target is far from
        # the image center.
        #
        # This prevents the drone from flying past the target
        # while simultaneously turning.
        # ---------------------------------------------------------

        if abs(error_x) > self.forward_alignment_limit:

            vx = 0.0

        else:

            vx = self.dist_pid.step(
                distance_error,
                dt
            )

        # ---------------------------------------------------------
        # Remember direction for target reacquisition.
        # ---------------------------------------------------------

        if abs(error_x) > self.horizontal_deadband:

            self.last_seen_dir = (
                -1.0 if error_x > 0 else 1.0
            )

        self.last_seen_vx = vx

        return {
            "vx": float(vx),
            "vy": 0.0,
            "vz": float(vz),
            "yaw_rate": float(yaw_rate),
        }