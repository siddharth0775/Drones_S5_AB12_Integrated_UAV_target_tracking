from pymavlink import mavutil
import time


PX4_CONNECTION = "udp:127.0.0.1:14540"


def connect_to_px4():
    print("Connecting to PX4...")

    vehicle = mavutil.mavlink_connection(
        PX4_CONNECTION
    )

    print("Waiting for PX4 heartbeat...")
    vehicle.wait_heartbeat()

    print("Connected to PX4!")
    print(f"System ID: {vehicle.target_system}")
    print(f"Component ID: {vehicle.target_component}")

    return vehicle


def arm(vehicle):
    print("\nArming drone...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # Arm
        0,      # Force
        0, 0, 0, 0, 0
    )

    message = vehicle.recv_match(
        type="COMMAND_ACK",
        blocking=True,
        timeout=5
    )

    if message:
        print(f"Arm response: {message}")

    time.sleep(2)


def takeoff(vehicle, altitude=2.0):
    print(f"\nRequesting takeoff to {altitude} meters...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude
    )

    message = vehicle.recv_match(
        type="COMMAND_ACK",
        blocking=True,
        timeout=5
    )

    if message:
        print(f"Takeoff response: {message}")


def get_altitude(vehicle):
    message = vehicle.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=2
    )

    if message:
        return message.relative_alt / 1000.0

    return None


def land(vehicle):
    print("\nLanding...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0,
        0
    )

def send_velocity(vehicle, vx, vy, vz, duration=0.1):
    """
    Send a velocity command to PX4 in the local NED frame.

    vx: forward/backward velocity (m/s)
    vy: left/right velocity (m/s)
    vz: down/up velocity (m/s)

    NED convention:
        +X = North / forward
        +Y = East / right
        +Z = Down
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

        0, 0, 0,       # Position
        vx, vy, vz,    # Velocity
        0, 0, 0,       # Acceleration
        0, 0            # Yaw, yaw rate
    )

def main():
    vehicle = connect_to_px4()

    print("\nWaiting before arming...")
    time.sleep(2)

    arm(vehicle)

    time.sleep(2)

    takeoff(vehicle, altitude=2.0)

    print("\nMonitoring altitude...")

    start_time = time.time()

    while time.time() - start_time < 15:
        altitude = get_altitude(vehicle)

        if altitude is not None:
            print(f"Altitude: {altitude:.2f} m")

            if altitude >= 1.8:
                print("\nTarget altitude reached!")
                break

        time.sleep(0.5)

    print("\nHolding for 5 seconds...")
    time.sleep(5)

    land(vehicle)

    print("\nLanding command sent.")


if __name__ == "__main__":
    main()
