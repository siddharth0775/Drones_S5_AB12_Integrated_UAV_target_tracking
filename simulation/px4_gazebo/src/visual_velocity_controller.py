import time
from pymavlink import mavutil


# ============================================================
# CONFIGURATION
# ============================================================

CONNECTION = "udp:127.0.0.1:14540"

SETPOINT_RATE = 20.0
DT = 1.0 / SETPOINT_RATE

# Camera
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

IMAGE_CENTER_X = IMAGE_WIDTH / 2.0

# Controller
KP = 1.2

MAX_SPEED = 1.0

DEADZONE_PIXELS = 30


# ============================================================
# CONNECT TO PX4
# ============================================================

def connect_to_px4():

    print("==========================================")
    print("     VISUAL VELOCITY CONTROLLER")
    print("==========================================")

    print("\nConnecting to PX4...")

    vehicle = mavutil.mavlink_connection(CONNECTION)

    print("Waiting for heartbeat...")

    vehicle.wait_heartbeat()

    print("Connected!")

    print(
        f"System ID: {vehicle.target_system}, "
        f"Component ID: {vehicle.target_component}"
    )

    return vehicle


# ============================================================
# SEND POSITION SETPOINT
# ============================================================

def send_position_setpoint(vehicle, x, y, z):

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
# SEND VELOCITY SETPOINT
# ============================================================

def send_velocity_setpoint(vehicle, vx, vy, vz):

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

        vx,
        vy,
        vz,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# ARM
# ============================================================

def arm_vehicle(vehicle):

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

def land_vehicle(vehicle):

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

    print("Landing command sent.")


# ============================================================
# PIXEL ERROR → VELOCITY
# ============================================================

def calculate_velocity(error_x):

    # Deadzone
    if abs(error_x) < DEADZONE_PIXELS:
        return 0.0

    # Normalize pixel error to [-1, +1]
    normalized_error = error_x / IMAGE_CENTER_X

    # Proportional controller
    velocity = KP * normalized_error

    # Limit velocity
    velocity = max(
        -MAX_SPEED,
        min(MAX_SPEED, velocity)
    )

    return velocity


# ============================================================
# DEMONSTRATION TARGET
# ============================================================

def simulated_target():

    """
    Temporary simulated target.

    This allows us to test the complete controller
    BEFORE connecting the real red detector.

    The target moves through several positions.
    """

    sequence = [

        # target_x, duration
        (640, 3),    # center
        (800, 4),    # right
        (640, 3),    # center
        (480, 4),    # left
        (640, 3),    # center
    ]

    return sequence


# ============================================================
# MAIN
# ============================================================

def main():

    vehicle = connect_to_px4()

    # ========================================================
    # INITIAL POSITION SETPOINTS
    # ========================================================

    print("\nPreparing Offboard setpoints...")

    print("Streaming initial position setpoints...")

    for _ in range(60):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -2.0
        )

        time.sleep(DT)

    print("Setpoint stream ready.")

    # ========================================================
    # OFFBOARD
    # ========================================================

    print("\nRequesting OFFBOARD...")

    vehicle.set_mode("OFFBOARD")

    time.sleep(1)

    print("OFFBOARD requested.")

    # Continue streaming

    for _ in range(20):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -2.0
        )

        time.sleep(DT)

    # ========================================================
    # ARM
    # ========================================================

    arm_vehicle(vehicle)

    # ========================================================
    # TAKEOFF / STABILIZATION
    # ========================================================

    print("\nTaking off to approximately 2 meters...")

    start = time.time()

    while time.time() - start < 6:

        send_position_setpoint(
            vehicle,
            0,
            0,
            -2.0
        )

        time.sleep(DT)

    print("Drone should now be hovering.")

    # ========================================================
    # VISUAL CONTROLLER TEST
    # ========================================================

    print("\n==========================================")
    print("STARTING VISUAL VELOCITY CONTROL")
    print("==========================================")

    sequence = simulated_target()

    for target_x, duration in sequence:

        print(
            f"\nSimulated target X = {target_x}"
        )

        start = time.time()

        while time.time() - start < duration:

            # -----------------------------------------------
            # Calculate pixel error
            # -----------------------------------------------

            error_x = target_x - IMAGE_CENTER_X

            # -----------------------------------------------
            # Convert error to velocity
            # -----------------------------------------------

            vx = calculate_velocity(error_x)

            # -----------------------------------------------
            # Send velocity
            # -----------------------------------------------

            send_velocity_setpoint(
                vehicle,
                vx,
                0.0,
                0.0
            )

            print(
                f"\rTarget X={target_x:4.0f} "
                f"Error={error_x:+5.0f} "
                f"VX={vx:+.2f} m/s",
                end=""
            )

            time.sleep(DT)

        print()

    # ========================================================
    # STOP
    # ========================================================

    print("\nStopping horizontal movement...")

    start = time.time()

    while time.time() - start < 3:

        send_velocity_setpoint(
            vehicle,
            0.0,
            0.0,
            0.0
        )

        time.sleep(DT)

    # ========================================================
    # LAND
    # ========================================================

    land_vehicle(vehicle)

    time.sleep(5)

    print("\n==========================================")
    print("VISUAL VELOCITY TEST COMPLETE")
    print("==========================================")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("\n\nEmergency stop requested.")

    except Exception as e:

        print("\nERROR:")
        print(e)
