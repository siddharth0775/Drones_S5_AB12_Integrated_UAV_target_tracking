import time
import math
import cv2
import numpy as np

from pymavlink import mavutil

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ============================================================
# CONFIGURATION
# ============================================================

PX4_CONNECTION = "udp:127.0.0.1:14540"

CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)

# ------------------------------------------------------------
# Flight
# ------------------------------------------------------------

TAKEOFF_HEIGHT = 3.0

SETPOINT_RATE = 20.0
DT = 1.0 / SETPOINT_RATE

# ------------------------------------------------------------
# Camera
# ------------------------------------------------------------

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

FX = 539.9363327
FY = 539.9363708

CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# ------------------------------------------------------------
# Tracking
# ------------------------------------------------------------

TARGET_DISTANCE = 3.0

MIN_TARGET_AREA = 100

# Maximum amount a position target is allowed
# to change during one control cycle.
MAX_POSITION_STEP = 0.08

# Smooth target position.
POSITION_FILTER = 0.35

# Ignore extremely tiny angular errors.
ANGLE_DEADBAND = 1.0

# ------------------------------------------------------------
# Red detection
# ------------------------------------------------------------

LOWER_RED_1 = np.array([0, 100, 80])
UPPER_RED_1 = np.array([10, 255, 255])

LOWER_RED_2 = np.array([170, 100, 80])
UPPER_RED_2 = np.array([180, 255, 255])


# ============================================================
# CAMERA / TARGET TRACKER
# ============================================================

class CameraTracker:

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

        mask1 = cv2.inRange(
            hsv,
            LOWER_RED_1,
            UPPER_RED_1
        )

        mask2 = cv2.inRange(
            hsv,
            LOWER_RED_2,
            UPPER_RED_2
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
# PX4 CONTROLLER
# ============================================================

class PX4Controller:

    def __init__(self):

        print("\nConnecting to PX4...")

        self.vehicle = mavutil.mavlink_connection(
            PX4_CONNECTION
        )

        print("Waiting for PX4 heartbeat...")

        self.vehicle.wait_heartbeat()

        print(
            f"Connected to PX4 "
            f"(system={self.vehicle.target_system}, "
            f"component={self.vehicle.target_component})"
        )


    # --------------------------------------------------------
    # POSITION SETPOINT
    # --------------------------------------------------------

    def send_position(self, x, y, z):

        # Position only.
        #
        # Ignore:
        # velocity
        # acceleration
        # yaw
        # yaw rate

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
    # GET LOCAL POSITION
    # --------------------------------------------------------

    def get_local_position(self):

        msg = self.vehicle.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=False
        )

        if msg is None:

            return None

        return (
            float(msg.x),
            float(msg.y),
            float(msg.z)
        )


    # --------------------------------------------------------
    # GET YAW
    # --------------------------------------------------------

    def get_yaw(self):

        msg = self.vehicle.recv_match(
            type="ATTITUDE",
            blocking=False
        )

        if msg is None:

            return None

        return float(msg.yaw)


    # --------------------------------------------------------
    # ARM
    # --------------------------------------------------------

    def arm(self):

        print("\nArming...")

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
    # LAND
    # --------------------------------------------------------

    def land(self):

        print("\nLANDING...")

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

        print("Land command sent.")


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


# ============================================================
# MATH
# ============================================================

def calculate_angles(target):

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
        angle_x,
        angle_y,
        angle_x_deg,
        angle_y_deg
    )


# ============================================================
# MAIN TRACKING MATH
# ============================================================

def target_relative_position(
    distance,
    angle_x,
    angle_y
):
    """
    Camera coordinate system:

        X = forward
        Y = right
        Z = down

    The target is approximately:

        x = distance * cos(vertical)
                         * cos(horizontal)

        y = distance * cos(vertical)
                         * sin(horizontal)

        z = distance * sin(vertical)
    """

    x = (
        distance
        * math.cos(angle_y)
        * math.cos(angle_x)
    )

    y = (
        distance
        * math.cos(angle_y)
        * math.sin(angle_x)
    )

    z = (
        distance
        * math.sin(angle_y)
    )

    return x, y, z


# ============================================================
# BODY → LOCAL NED
# ============================================================

def body_to_local(
    forward,
    right,
    down,
    yaw
):

    north = (
        forward * math.cos(yaw)
        - right * math.sin(yaw)
    )

    east = (
        forward * math.sin(yaw)
        + right * math.cos(yaw)
    )

    return north, east, down


# ============================================================
# LIMIT POSITION CHANGE
# ============================================================

def limit_position_step(
    current,
    desired,
    maximum_step
):

    difference = desired - current

    if abs(difference) > maximum_step:

        difference = (
            math.copysign(
                maximum_step,
                difference
            )
        )

    return current + difference


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("==============================================")
    print("       PX4 OBJECT TRACKING DRONE")
    print("==============================================")
    print()
    print("Camera tracking + angle estimation")
    print("Position-based PX4 control")
    print("Target distance: 3 meters")
    print()


    # ========================================================
    # INITIALIZE
    # ========================================================

    camera = CameraTracker()

    px4 = PX4Controller()


    # ========================================================
    # WAIT FOR LOCAL POSITION
    # ========================================================

    print("\nWaiting for local position...")

    initial_position = None

    while initial_position is None:

        initial_position = (
            px4.get_local_position()
        )

        time.sleep(0.1)

    home_x = initial_position[0]
    home_y = initial_position[1]

    print(
        f"Home position: "
        f"x={home_x:.2f}, "
        f"y={home_y:.2f}"
    )


    # ========================================================
    # INITIAL OFFBOARD SETPOINT STREAM
    # ========================================================

    print("\nPreparing position setpoints...")

    for _ in range(80):

        px4.send_position(
            home_x,
            home_y,
            -TAKEOFF_HEIGHT
        )

        time.sleep(DT)


    # ========================================================
    # OFFBOARD
    # ========================================================

    px4.offboard()


    # Keep sending setpoints
    for _ in range(20):

        px4.send_position(
            home_x,
            home_y,
            -TAKEOFF_HEIGHT
        )

        time.sleep(DT)


    # ========================================================
    # ARM
    # ========================================================

    px4.arm()


    # ========================================================
    # TAKEOFF
    # ========================================================

    print()
    print("==============================================")
    print("TAKING OFF")
    print("==============================================")

    takeoff_start = time.time()

    while time.time() - takeoff_start < 8.0:

        px4.send_position(
            home_x,
            home_y,
            -TAKEOFF_HEIGHT
        )

        time.sleep(DT)


    print("\nDrone should now be hovering at 3 meters.")


    # ========================================================
    # WAIT FOR TARGET
    # ========================================================

    print()
    print("==============================================")
    print("WAITING FOR RED TARGET")
    print("==============================================")
    print()
    print("Move the red object into the camera.")
    print("The first stable detection becomes")
    print("the 3-meter reference.")
    print()


    reference_height = None

    stable_count = 0

    while reference_height is None:

        if camera.latest_frame is None:

            px4.send_position(
                home_x,
                home_y,
                -TAKEOFF_HEIGHT
            )

            time.sleep(DT)

            continue


        frame = camera.latest_frame.copy()

        target, mask = camera.detect_target(
            frame
        )


        # Keep hovering

        px4.send_position(
            home_x,
            home_y,
            -TAKEOFF_HEIGHT
        )


        if target is not None:

            stable_count += 1

            print(
                f"\rTarget detected "
                f"height={target['h']} "
                f"stable={stable_count}/20",
                end=""
            )

            if stable_count >= 20:

                reference_height = (
                    float(target["h"])
                )

        else:

            stable_count = 0


        # Display

        cv2.imshow(
            "OBJECT TRACKING DRONE",
            frame
        )

        cv2.waitKey(1)

        time.sleep(DT)


    print()
    print()
    print(
        f"3-meter reference target height: "
        f"{reference_height:.1f} pixels"
    )


    # ========================================================
    # TRACKING INITIALIZATION
    # ========================================================

    local_position = (
        px4.get_local_position()
    )

    if local_position is None:

        print("Could not read local position.")

        px4.land()

        return


    desired_x = local_position[0]
    desired_y = local_position[1]
    desired_z = -TAKEOFF_HEIGHT


    # ========================================================
    # TRACKING LOOP
    # ========================================================

    print()
    print("==============================================")
    print("          TRACKING STARTED")
    print("==============================================")
    print()
    print("Move the red object around.")
    print("The drone should follow it.")
    print()
    print("Press Q to land.")
    print()


    last_print = time.time()


    try:

        while True:

            # ------------------------------------------------
            # CAMERA
            # ------------------------------------------------

            if camera.latest_frame is None:

                px4.send_position(
                    desired_x,
                    desired_y,
                    desired_z
                )

                time.sleep(DT)

                continue


            frame = camera.latest_frame.copy()


            # ------------------------------------------------
            # DETECT TARGET
            # ------------------------------------------------

            target, mask = (
                camera.detect_target(frame)
            )


            # ------------------------------------------------
            # GET DRONE POSITION
            # ------------------------------------------------

            local_position = (
                px4.get_local_position()
            )

            yaw = px4.get_yaw()


            if (
                local_position is None
                or yaw is None
            ):

                px4.send_position(
                    desired_x,
                    desired_y,
                    desired_z
                )

                time.sleep(DT)

                continue


            drone_x = local_position[0]
            drone_y = local_position[1]
            drone_z = local_position[2]


            # =================================================
            # TARGET FOUND
            # =================================================

            if target is not None:

                (
                    error_x,
                    error_y,
                    angle_x,
                    angle_y,
                    angle_x_deg,
                    angle_y_deg
                ) = calculate_angles(
                    target
                )


                # ------------------------------------------------
                # ESTIMATE DISTANCE
                # ------------------------------------------------
                #
                # At the beginning:
                #
                # reference_height = target height at 3 m
                #
                # Therefore:
                #
                # distance =
                # 3 * reference_height / current_height
                #
                # This is an approximate monocular range estimate.
                #

                current_height = max(
                    float(target["h"]),
                    1.0
                )

                estimated_distance = (
                    TARGET_DISTANCE
                    * reference_height
                    / current_height
                )


                # Keep estimate reasonable

                estimated_distance = max(
                    1.0,
                    min(
                        estimated_distance,
                        10.0
                    )
                )


                # ------------------------------------------------
                # TARGET POSITION IN CAMERA/BODY FRAME
                # ------------------------------------------------

                forward, right, down = (
                    target_relative_position(
                        estimated_distance,
                        angle_x,
                        angle_y
                    )
                )


                # ------------------------------------------------
                # TARGET POSITION IN LOCAL NED
                # ------------------------------------------------

                target_north, target_east, target_down = (
                    body_to_local(
                        forward,
                        right,
                        down,
                        yaw
                    )
                )


                target_world_x = (
                    drone_x + target_north
                )

                target_world_y = (
                    drone_y + target_east
                )


                # ------------------------------------------------
                # DESIRED DRONE POSITION
                # ------------------------------------------------
                #
                # We want:
                #
                # target = 3 m directly in front
                #
                # Therefore the drone's desired position is
                # 3 meters behind the target along its current
                # heading.
                #
                # ------------------------------------------------

                desired_forward = (
                    TARGET_DISTANCE
                )

                desired_right = 0.0
                desired_down = 0.0


                desired_offset_x, desired_offset_y, _ = (
                    body_to_local(
                        desired_forward,
                        desired_right,
                        desired_down,
                        yaw
                    )
                )


                raw_desired_x = (
                    target_world_x
                    - desired_offset_x
                )

                raw_desired_y = (
                    target_world_y
                    - desired_offset_y
                )


                # ------------------------------------------------
                # HEIGHT
                # ------------------------------------------------
                #
                # Since the object is assumed to move on the
                # ground, keeping approximately 3 m altitude
                # maintains the required distance.
                #

                raw_desired_z = (
                    -TAKEOFF_HEIGHT
                )


                # ------------------------------------------------
                # SMOOTH POSITION TARGET
                # ------------------------------------------------

                filtered_x = (
                    desired_x
                    + POSITION_FILTER
                    * (raw_desired_x - desired_x)
                )

                filtered_y = (
                    desired_y
                    + POSITION_FILTER
                    * (raw_desired_y - desired_y)
                )

                filtered_z = (
                    desired_z
                    + POSITION_FILTER
                    * (raw_desired_z - desired_z)
                )


                # ------------------------------------------------
                # LIMIT STEP
                # ------------------------------------------------

                desired_x = (
                    limit_position_step(
                        desired_x,
                        filtered_x,
                        MAX_POSITION_STEP
                    )
                )

                desired_y = (
                    limit_position_step(
                        desired_y,
                        filtered_y,
                        MAX_POSITION_STEP
                    )
                )

                desired_z = (
                    limit_position_step(
                        desired_z,
                        filtered_z,
                        MAX_POSITION_STEP
                    )
                )


                # ------------------------------------------------
                # SEND PX4 POSITION
                # ------------------------------------------------

                px4.send_position(
                    desired_x,
                    desired_y,
                    desired_z
                )


                # =================================================
                # DRAW TRACKING INFORMATION
                # =================================================

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
                    2
                )


                cv2.circle(
                    frame,
                    (target_cx, target_cy),
                    7,
                    (255, 0, 0),
                    -1
                )


                cv2.circle(
                    frame,
                    (int(CX), int(CY)),
                    7,
                    (255, 255, 0),
                    -1
                )


                cv2.line(
                    frame,
                    (int(CX), int(CY)),
                    (target_cx, target_cy),
                    (0, 255, 255),
                    2
                )


                # ------------------------------------------------
                # DISPLAY
                # ------------------------------------------------

                cv2.putText(
                    frame,
                    "TRACKING",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )


                cv2.putText(
                    frame,
                    f"Angle X: {angle_x_deg:+.2f} deg",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Angle Y: {angle_y_deg:+.2f} deg",
                    (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Distance: {estimated_distance:.2f} m",
                    (20, 130),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Target size: {target['h']} px",
                    (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2
                )


                # ------------------------------------------------
                # TERMINAL
                # ------------------------------------------------

                if time.time() - last_print > 0.25:

                    print(
                        f"\r"
                        f"Angle=({angle_x_deg:+6.1f},"
                        f"{angle_y_deg:+6.1f}) | "
                        f"Distance={estimated_distance:4.2f}m | "
                        f"Target=({target_cx:4d},"
                        f"{target_cy:4d}) | "
                        f"Desired=({desired_x:+5.2f},"
                        f"{desired_y:+5.2f},"
                        f"{desired_z:+5.2f})",
                        end=""
                    )

                    last_print = time.time()


            # =================================================
            # TARGET LOST
            # =================================================

            else:

                # IMPORTANT:
                #
                # Do NOT fly blindly when the target disappears.
                #
                # Simply hold the last known position.

                px4.send_position(
                    desired_x,
                    desired_y,
                    desired_z
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
            # DISPLAY
            # ------------------------------------------------

            cv2.imshow(
                "OBJECT TRACKING DRONE",
                frame
            )

            cv2.imshow(
                "RED MASK",
                mask
            )


            key = cv2.waitKey(1) & 0xFF


            if key == ord("q"):

                print("\n\nQ pressed.")

                break


            time.sleep(DT)


    except KeyboardInterrupt:

        print("\n\nKeyboard interrupt.")


    finally:

        print("\n")
        print("==============================================")
        print("LANDING")
        print("==============================================")


        # Send the current position for a short time
        # before landing.

        for _ in range(20):

            px4.send_position(
                desired_x,
                desired_y,
                desired_z
            )

            time.sleep(DT)


        px4.land()

        time.sleep(8)

        cv2.destroyAllWindows()

        print("\nTracking mission finished.")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
