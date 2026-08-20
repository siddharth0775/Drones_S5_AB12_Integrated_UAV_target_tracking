import time
import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from pymavlink import mavutil


# ============================================================
# CONFIGURATION
# ============================================================

CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)

PX4_CONNECTION = "udp:127.0.0.1:14540"


# ------------------------------------------------------------
# CAMERA
# ------------------------------------------------------------

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

FX = 539.9363327
FY = 539.9363708

CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0


# ------------------------------------------------------------
# CONTROLLER
# ------------------------------------------------------------

KP_HORIZONTAL = 0.8
KP_VERTICAL = 0.8

MAX_HORIZONTAL_SPEED = 1.0
MAX_FORWARD_SPEED = 1.0

DEADBAND_PIXELS = 35


# ------------------------------------------------------------
# TAKEOFF
# ------------------------------------------------------------

TAKEOFF_ALTITUDE = 2.5

SETPOINT_RATE = 20.0
DT = 1.0 / SETPOINT_RATE


# ============================================================
# EASY DIRECTION REVERSAL
# ============================================================
#
# If the drone moves in the WRONG direction, change:
#
# +1  -> normal
# -1  -> reverse
#
# ------------------------------------------------------------

RIGHT_SIGN = 1.0

FORWARD_SIGN = 1.0


# ============================================================
# PX4 CONNECTION
# ============================================================

def connect_px4():

    print("==========================================")
    print("       CENTER TRACKING DRONE")
    print("==========================================")

    print("\nConnecting to PX4...")

    vehicle = mavutil.mavlink_connection(
        PX4_CONNECTION
    )

    print("Waiting for PX4 heartbeat...")

    vehicle.wait_heartbeat()

    print("PX4 connected!")

    print(
        f"System: {vehicle.target_system} "
        f"Component: {vehicle.target_component}"
    )

    return vehicle


# ============================================================
# SEND POSITION SETPOINT
# ============================================================

def send_position_setpoint(
    vehicle,
    x,
    y,
    z
):

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    vehicle.mav.set_position_target_local_ned_send(

        int(time.time() * 1000) & 0xFFFFFFFF,

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        x,
        y,
        z,

        0,
        0,
        0,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# SEND VELOCITY
# ============================================================

def send_velocity(
    vehicle,
    forward,
    right
):

    """
    LOCAL_NED

    forward -> X
    right   -> Y

    Z = 0

    Therefore altitude does NOT change.
    """

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    vehicle.mav.set_position_target_local_ned_send(

        int(time.time() * 1000) & 0xFFFFFFFF,

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        0,
        0,
        0,

        forward,
        right,
        0.0,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# ARM
# ============================================================

def arm(vehicle):

    print("\nArming...")

    vehicle.mav.command_long_send(

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

        0,

        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

    time.sleep(2)

    print("Arm command sent.")


# ============================================================
# LAND
# ============================================================

def land(vehicle):

    print("\nLanding...")

    vehicle.mav.command_long_send(

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_LAND,

        0,

        0,
        0,
        0,
        0,
        0,
        0,
        0
    )


# ============================================================
# RED TARGET TRACKER
# ============================================================

class RedTracker:

    def __init__(self):

        self.node = Node()

        self.latest_frame = None

        success = self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.camera_callback
        )

        if not success:

            raise RuntimeError(
                "Could not subscribe to Gazebo camera."
            )

        print("Camera subscription successful!")


    # --------------------------------------------------------
    # CAMERA CALLBACK
    # --------------------------------------------------------

    def camera_callback(self, msg):

        try:

            width = msg.width
            height = msg.height

            data = np.frombuffer(
                msg.data,
                dtype=np.uint8
            )

            expected = width * height * 3

            if data.size != expected:

                return

            frame = data.reshape(
                (height, width, 3)
            )

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

            self.latest_frame = frame

        except Exception as e:

            print(
                "\nCamera error:",
                e
            )


    # --------------------------------------------------------
    # RED DETECTION
    # --------------------------------------------------------

    def detect_red(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower1 = np.array(
            [0, 100, 80]
        )

        upper1 = np.array(
            [10, 255, 255]
        )

        lower2 = np.array(
            [170, 100, 80]
        )

        upper2 = np.array(
            [180, 255, 255]
        )

        mask1 = cv2.inRange(
            hsv,
            lower1,
            upper1
        )

        mask2 = cv2.inRange(
            hsv,
            lower2,
            upper2
        )

        mask = mask1 | mask2

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:

            return None, mask

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(
            largest
        )

        if area < 100:

            return None, mask

        x, y, w, h = cv2.boundingRect(
            largest
        )

        center_x = x + w // 2
        center_y = y + h // 2

        target = {

            "x": x,
            "y": y,

            "w": w,
            "h": h,

            "center_x": center_x,
            "center_y": center_y,

            "area": area
        }

        return target, mask


# ============================================================
# CALCULATE TRACKING VELOCITY
# ============================================================

def calculate_tracking_velocity(target):

    target_x = target["center_x"]
    target_y = target["center_y"]

    # --------------------------------------------------------
    # IMAGE ERROR
    # --------------------------------------------------------

    error_x = target_x - CX
    error_y = target_y - CY


    # --------------------------------------------------------
    # DEADZONE
    # --------------------------------------------------------

    if abs(error_x) < DEADBAND_PIXELS:

        error_x = 0.0

    if abs(error_y) < DEADBAND_PIXELS:

        error_y = 0.0


    # --------------------------------------------------------
    # NORMALIZE BY CAMERA FOCAL LENGTH
    # --------------------------------------------------------

    normalized_x = error_x / FX
    normalized_y = error_y / FY


    # --------------------------------------------------------
    # CONTROLLER
    # --------------------------------------------------------

    #
    # Target LEFT / RIGHT
    #        ↓
    # right velocity
    #

    right_velocity = (
        KP_HORIZONTAL *
        normalized_x
    )


    #
    # Target UP / DOWN in camera
    #        ↓
    # forward / backward
    #

    forward_velocity = (
        KP_VERTICAL *
        normalized_y
    )


    # --------------------------------------------------------
    # APPLY SIGN
    # --------------------------------------------------------

    right_velocity *= RIGHT_SIGN

    forward_velocity *= FORWARD_SIGN


    # --------------------------------------------------------
    # LIMIT
    # --------------------------------------------------------

    right_velocity = max(
        -MAX_HORIZONTAL_SPEED,
        min(
            MAX_HORIZONTAL_SPEED,
            right_velocity
        )
    )

    forward_velocity = max(
        -MAX_FORWARD_SPEED,
        min(
            MAX_FORWARD_SPEED,
            forward_velocity
        )
    )


    return (
        forward_velocity,
        right_velocity,
        error_x,
        error_y
    )


# ============================================================
# MAIN
# ============================================================

def main():

    vehicle = connect_px4()

    tracker = RedTracker()


    # ========================================================
    # INITIAL SETPOINT STREAM
    # ========================================================

    print("\nPreparing takeoff...")

    for _ in range(60):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(DT)


    # ========================================================
    # OFFBOARD
    # ========================================================

    print("Entering OFFBOARD...")

    vehicle.set_mode(
        "OFFBOARD"
    )

    time.sleep(1)


    # Keep streaming position
    # before arming.

    for _ in range(20):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(DT)


    # ========================================================
    # ARM
    # ========================================================

    arm(vehicle)


    # ========================================================
    # TAKEOFF / HOVER
    # ========================================================

    print(
        f"\nTaking off to "
        f"{TAKEOFF_ALTITUDE:.1f} m..."
    )

    start = time.time()

    while time.time() - start < 6:

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(DT)


    print("\n==========================================")
    print("       CENTER TRACKING ACTIVE")
    print("==========================================")

    print(
        "\nMove the red object around."
    )

    print(
        "The drone will try to put the "
        "object at the camera center."
    )

    print(
        "\nPress Q to land."
    )


    # ========================================================
    # TRACKING LOOP
    # ========================================================

    try:

        while True:

            # ------------------------------------------------
            # CAMERA NOT READY
            # ------------------------------------------------

            if tracker.latest_frame is None:

                send_velocity(
                    vehicle,
                    0.0,
                    0.0
                )

                time.sleep(DT)

                continue


            frame = tracker.latest_frame.copy()

            height, width = frame.shape[:2]


            # ------------------------------------------------
            # DETECT TARGET
            # ------------------------------------------------

            target, mask = tracker.detect_red(
                frame
            )


            # ------------------------------------------------
            # CAMERA CENTER
            # ------------------------------------------------

            camera_cx = width // 2
            camera_cy = height // 2

            cv2.circle(
                frame,
                (
                    camera_cx,
                    camera_cy
                ),
                8,
                (255, 255, 0),
                -1
            )


            # =================================================
            # TARGET FOUND
            # =================================================

            if target is not None:

                # ---------------------------------------------
                # Calculate velocity
                # ---------------------------------------------

                (
                    forward_velocity,
                    right_velocity,
                    error_x,
                    error_y
                ) = calculate_tracking_velocity(
                    target
                )


                # ---------------------------------------------
                # SEND VELOCITY
                # ---------------------------------------------

                send_velocity(
                    vehicle,
                    forward_velocity,
                    right_velocity
                )


                # ---------------------------------------------
                # DRAW TARGET
                # ---------------------------------------------

                x = target["x"]
                y = target["y"]
                w = target["w"]
                h = target["h"]

                target_cx = target["center_x"]
                target_cy = target["center_y"]


                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    3
                )


                cv2.circle(
                    frame,
                    (
                        target_cx,
                        target_cy
                    ),
                    8,
                    (255, 0, 0),
                    -1
                )


                # Line showing error

                cv2.line(
                    frame,
                    (
                        camera_cx,
                        camera_cy
                    ),
                    (
                        target_cx,
                        target_cy
                    ),
                    (0, 255, 255),
                    2
                )


                # ---------------------------------------------
                # DISPLAY
                # ---------------------------------------------

                cv2.putText(
                    frame,
                    "TRACKING",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2
                )


                cv2.putText(
                    frame,
                    f"Error X: {error_x:+.0f}",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Error Y: {error_y:+.0f}",
                    (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Forward: {forward_velocity:+.2f} m/s",
                    (20, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Right: {right_velocity:+.2f} m/s",
                    (20, 175),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                print(
                    f"\r"
                    f"Error X={error_x:+6.0f} | "
                    f"Error Y={error_y:+6.0f} | "
                    f"Forward={forward_velocity:+.2f} | "
                    f"Right={right_velocity:+.2f}",
                    end=""
                )


            # =================================================
            # TARGET LOST
            # =================================================

            else:

                # IMPORTANT:
                # If we cannot see the target,
                # DO NOT continue moving.

                send_velocity(
                    vehicle,
                    0.0,
                    0.0
                )


                cv2.putText(
                    frame,
                    "TARGET LOST - HOLDING",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )


            # ------------------------------------------------
            # SHOW CAMERA
            # ------------------------------------------------

            cv2.imshow(
                "Tracker Drone Camera",
                frame
            )

            cv2.imshow(
                "Red Detection Mask",
                mask
            )


            # ------------------------------------------------
            # KEYBOARD
            # ------------------------------------------------

            key = cv2.waitKey(1) & 0xFF


            if key == ord("q"):

                print(
                    "\n\nQ pressed."
                )

                break


            time.sleep(DT)


    except KeyboardInterrupt:

        print(
            "\n\nKeyboard interrupt."
        )


    finally:

        # ----------------------------------------------------
        # STOP MOVEMENT
        # ----------------------------------------------------

        print(
            "\nStopping horizontal movement..."
        )

        for _ in range(30):

            send_velocity(
                vehicle,
                0.0,
                0.0
            )

            time.sleep(DT)


        # ----------------------------------------------------
        # LAND
        # ----------------------------------------------------

        land(vehicle)

        time.sleep(8)

        cv2.destroyAllWindows()

        print(
            "\nTracking test complete."
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
