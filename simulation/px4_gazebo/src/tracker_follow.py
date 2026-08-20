import time
import math
import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from pymavlink import mavutil


# ============================================================
# CONFIGURATION
# ============================================================

PX4_CONNECTION = "udp:127.0.0.1:14540"

CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)

# Camera
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

FX = 539.9363327
FY = 539.9363708

CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0


# ============================================================
# TARGET / DISTANCE
# ============================================================

DESIRED_DISTANCE = 3.0

# Diameter of the red target in metres.
#
# CHANGE THIS if your target sphere is not 0.20 m.
TARGET_DIAMETER = 0.20

# Camera focal length + target size are used to estimate distance:
#
# distance = focal_length * real_width / pixel_width

MIN_TARGET_AREA = 100


# ============================================================
# DRONE CONTROL
# ============================================================

TAKEOFF_ALTITUDE = 2.0

CONTROL_RATE = 20.0
DT = 1.0 / CONTROL_RATE

MAX_FORWARD_SPEED = 1.0
MAX_SIDE_SPEED = 1.0

# Horizontal angle controller
KP_FORWARD = 0.8
KP_SIDE = 1.2

# Distance controller
KP_DISTANCE = 0.8

# Deadbands
ANGLE_DEADBAND = 2.0
DISTANCE_DEADBAND = 0.15


# ============================================================
# CLAMP
# ============================================================

def clamp(value, minimum, maximum):

    return max(
        minimum,
        min(value, maximum)
    )


# ============================================================
# CAMERA + TARGET TRACKER
# ============================================================

class TargetTracker:

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
                "Failed to subscribe to Gazebo camera"
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

    def detect_target(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower_red_1 = np.array(
            [0, 100, 80]
        )

        upper_red_1 = np.array(
            [10, 255, 255]
        )

        lower_red_2 = np.array(
            [170, 100, 80]
        )

        upper_red_2 = np.array(
            [180, 255, 255]
        )

        mask1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
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

        area = cv2.contourArea(largest)

        if area < MIN_TARGET_AREA:
            return None, mask

        x, y, w, h = cv2.boundingRect(
            largest
        )

        center_x = x + w / 2.0
        center_y = y + h / 2.0

        return {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "center_x": center_x,
            "center_y": center_y,
            "area": area
        }, mask


    # --------------------------------------------------------
    # ANGLE CALCULATION
    # --------------------------------------------------------

    def calculate_angles(self, target):

        error_x = (
            target["center_x"] - CX
        )

        error_y = (
            target["center_y"] - CY
        )

        angle_x = math.atan2(
            error_x,
            FX
        )

        angle_y = math.atan2(
            error_y,
            FY
        )

        angle_x_deg = math.degrees(
            angle_x
        )

        angle_y_deg = math.degrees(
            angle_y
        )

        return (
            error_x,
            error_y,
            angle_x_deg,
            angle_y_deg
        )


    # --------------------------------------------------------
    # DISTANCE ESTIMATION
    # --------------------------------------------------------

    def estimate_distance(self, target):

        pixel_width = max(
            target["w"],
            1
        )

        distance = (
            FX *
            TARGET_DIAMETER /
            pixel_width
        )

        return distance


# ============================================================
# PX4 CONTROLLER
# ============================================================

class DroneController:

    def __init__(self):

        print("\nConnecting to PX4...")

        self.vehicle = mavutil.mavlink_connection(
            PX4_CONNECTION
        )

        print("Waiting for heartbeat...")

        self.vehicle.wait_heartbeat()

        print("PX4 connected!")

        print(
            f"System ID: "
            f"{self.vehicle.target_system}"
        )

        print(
            f"Component ID: "
            f"{self.vehicle.target_component}"
        )


    # --------------------------------------------------------
    # POSITION SETPOINT
    # --------------------------------------------------------

    def send_position(
        self,
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

        self.vehicle.mav.set_position_target_local_ned_send(

            int(time.time() * 1000) & 0xFFFFFFFF,

            self.vehicle.target_system,
            self.vehicle.target_component,

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


    # --------------------------------------------------------
    # VELOCITY SETPOINT
    # --------------------------------------------------------

    def send_velocity(
        self,
        vx,
        vy,
        vz=0.0
    ):

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

        self.vehicle.mav.set_position_target_local_ned_send(

            int(time.time() * 1000) & 0xFFFFFFFF,

            self.vehicle.target_system,
            self.vehicle.target_component,

            mavutil.mavlink.MAV_FRAME_LOCAL_NED,

            type_mask,

            0,
            0,
            0,

            vx,
            vy,
            vz,

            0,
            0,
            0,

            0,
            0
        )


    # --------------------------------------------------------
    # ARM
    # --------------------------------------------------------

    def arm(self):

        print("\nArming drone...")

        self.vehicle.mav.command_long_send(

            self.vehicle.target_system,
            self.vehicle.target_component,

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


    # --------------------------------------------------------
    # OFFBOARD
    # --------------------------------------------------------

    def offboard(self):

        print("\nRequesting OFFBOARD...")

        self.vehicle.set_mode(
            "OFFBOARD"
        )

        time.sleep(1)

        print("OFFBOARD requested.")


    # --------------------------------------------------------
    # TAKEOFF
    # --------------------------------------------------------

    def takeoff(self):

        print(
            f"\nTaking off to "
            f"{TAKEOFF_ALTITUDE:.1f} m..."
        )

        # PX4 needs setpoints before OFFBOARD.
        for _ in range(60):

            self.send_position(
                0,
                0,
                -TAKEOFF_ALTITUDE
            )

            time.sleep(DT)

        self.offboard()

        # Arm after OFFBOARD request
        self.arm()

        # Hold 2 m for stabilization
        start = time.time()

        while time.time() - start < 6:

            self.send_position(
                0,
                0,
                -TAKEOFF_ALTITUDE
            )

            time.sleep(DT)

        print(
            "\nTakeoff complete."
        )


    # --------------------------------------------------------
    # LAND
    # --------------------------------------------------------

    def land(self):

        print("\nLanding...")

        self.vehicle.mav.command_long_send(

            self.vehicle.target_system,
            self.vehicle.target_component,

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
# FOLLOW CONTROLLER
# ============================================================

def calculate_follow_velocity(
    angle_x_deg,
    distance
):

    # ========================================================
    # LEFT / RIGHT
    # ========================================================

    if abs(angle_x_deg) < ANGLE_DEADBAND:

        vy = 0.0

    else:

        vy = (
            KP_SIDE *
            math.radians(angle_x_deg)
        )

        vy = clamp(
            vy,
            -MAX_SIDE_SPEED,
            MAX_SIDE_SPEED
        )


    # ========================================================
    # FORWARD / BACKWARD
    # ========================================================

    distance_error = (
        distance -
        DESIRED_DISTANCE
    )

    if abs(distance_error) < DISTANCE_DEADBAND:

        vx = 0.0

    else:

        vx = (
            KP_DISTANCE *
            distance_error
        )

        vx = clamp(
            vx,
            -MAX_FORWARD_SPEED,
            MAX_FORWARD_SPEED
        )


    # ========================================================
    # NO VERTICAL CONTROL
    # ========================================================

    vz = 0.0

    return vx, vy, vz


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        "\n=============================================="
    )

    print(
        "        OBJECT TRACKING DRONE"
    )

    print(
        "=============================================="
    )

    print(
        "\nTarget distance:",
        DESIRED_DISTANCE,
        "m"
    )

    print(
        "Vertical tracking: DISABLED"
    )

    print(
        "Horizontal tracking: ENABLED"
    )


    # ========================================================
    # INITIALIZE
    # ========================================================

    tracker = TargetTracker()

    drone = DroneController()


    # ========================================================
    # TAKEOFF
    # ========================================================

    drone.takeoff()


    print(
        "\n=============================================="
    )

    print(
        "       TRACKING STARTED"
    )

    print(
        "=============================================="
    )

    print(
        "\nMove the red target using your"
    )

    print(
        "object_controller.py"
    )

    print(
        "\nPress Q in the camera window to land."
    )


    # ========================================================
    # TRACKING LOOP
    # ========================================================

    try:

        while True:

            if tracker.latest_frame is None:

                time.sleep(0.01)

                continue


            frame = tracker.latest_frame.copy()


            target, mask = tracker.detect_target(
                frame
            )


            # =================================================
            # TARGET FOUND
            # =================================================

            if target is not None:

                (
                    error_x,
                    error_y,
                    angle_x,
                    angle_y
                ) = tracker.calculate_angles(
                    target
                )


                distance = (
                    tracker.estimate_distance(
                        target
                    )
                )


                # ---------------------------------------------
                # CONTROLLER
                # ---------------------------------------------

                vx, vy, vz = (
                    calculate_follow_velocity(
                        angle_x,
                        distance
                    )
                )


                # ---------------------------------------------
                # SEND VELOCITY
                # ---------------------------------------------

                drone.send_velocity(
                    vx,
                    vy,
                    vz
                )


                # ---------------------------------------------
                # DRAW TARGET
                # ---------------------------------------------

                x = target["x"]
                y = target["y"]
                w = target["w"]
                h = target["h"]

                target_cx = int(
                    target["center_x"]
                )

                target_cy = int(
                    target["center_y"]
                )


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
                    7,
                    (255, 0, 0),
                    -1
                )


                # Camera center

                cv2.circle(
                    frame,
                    (
                        int(CX),
                        int(CY)
                    ),
                    7,
                    (255, 255, 0),
                    -1
                )


                # Error line

                cv2.line(
                    frame,

                    (
                        int(CX),
                        int(CY)
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
                    (20, 40),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.9,

                    (0, 255, 0),

                    2
                )


                cv2.putText(
                    frame,

                    f"Angle X: "
                    f"{angle_x:+.2f} deg",

                    (20, 80),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (255, 255, 255),

                    2
                )


                cv2.putText(
                    frame,

                    f"Angle Y: "
                    f"{angle_y:+.2f} deg",

                    (20, 110),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (255, 255, 255),

                    2
                )


                cv2.putText(
                    frame,

                    f"Distance: "
                    f"{distance:.2f} m",

                    (20, 140),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (255, 255, 255),

                    2
                )


                cv2.putText(
                    frame,

                    f"VX: "
                    f"{vx:+.2f} m/s",

                    (20, 180),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (255, 255, 255),

                    2
                )


                cv2.putText(
                    frame,

                    f"VY: "
                    f"{vy:+.2f} m/s",

                    (20, 210),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (255, 255, 255),

                    2
                )


                cv2.putText(
                    frame,

                    "Z CONTROL: OFF",

                    (20, 240),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.65,

                    (0, 255, 255),

                    2
                )


                print(
                    f"\r"
                    f"AngleX={angle_x:+6.2f}° | "
                    f"Distance={distance:.2f}m | "
                    f"VX={vx:+.2f} | "
                    f"VY={vy:+.2f}",
                    end=""
                )


            # =================================================
            # TARGET LOST
            # =================================================

            else:

                # IMPORTANT:
                # Stop horizontal movement if target disappears.

                drone.send_velocity(
                    0.0,
                    0.0,
                    0.0
                )


                cv2.putText(
                    frame,

                    "TARGET LOST - HOLDING",

                    (20, 50),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.9,

                    (0, 0, 255),

                    2
                )


            # =================================================
            # SHOW CAMERA
            # =================================================

            cv2.imshow(
                "TRACKER DRONE CAMERA",
                frame
            )


            cv2.imshow(
                "RED TARGET MASK",
                mask
            )


            key = (
                cv2.waitKey(1)
                & 0xFF
            )


            if key == ord("q"):

                break


            time.sleep(
                max(
                    0,
                    DT
                )
            )


    except KeyboardInterrupt:

        print(
            "\n\nStopping..."
        )


    finally:

        print(
            "\nStopping horizontal movement..."
        )

        for _ in range(20):

            drone.send_velocity(
                0.0,
                0.0,
                0.0
            )

            time.sleep(DT)


        drone.land()

        cv2.destroyAllWindows()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()
