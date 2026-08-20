import asyncio

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


# ============================================================
# SETTINGS
# ============================================================

TAKEOFF_TIME = 5

MAX_SPEED = 0.8          # m/s
MIN_SPEED = 0.0          # m/s

RAMP_TIME = 5            # seconds to accelerate/decelerate

HOLD_TIME = 3             # seconds at each target velocity

STOP_TIME = 3             # seconds stationary before landing


# ============================================================
# SEND ONE VELOCITY COMMAND
# ============================================================

async def send_velocity(drone, forward, right=0.0, down=0.0):

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            forward,
            right,
            down,
            0.0
        )
    )


# ============================================================
# RAMP VELOCITY SMOOTHLY
# ============================================================

async def ramp_velocity(drone, start_speed, end_speed, duration):

    print(
        f"Ramping velocity: "
        f"{start_speed:.2f} -> {end_speed:.2f} m/s"
    )

    update_rate = 20.0
    dt = 1.0 / update_rate

    steps = int(duration * update_rate)

    for i in range(steps + 1):

        progress = i / steps

        speed = (
            start_speed
            + (end_speed - start_speed) * progress
        )

        await send_velocity(drone, speed)

        await asyncio.sleep(dt)


# ============================================================
# HOLD VELOCITY
# ============================================================

async def hold_velocity(drone, speed, duration):

    print(
        f"Holding velocity: {speed:.2f} m/s "
        f"for {duration:.1f} seconds"
    )

    update_rate = 20.0
    dt = 1.0 / update_rate

    steps = int(duration * update_rate)

    for _ in range(steps):

        await send_velocity(drone, speed)

        await asyncio.sleep(dt)


# ============================================================
# MAIN
# ============================================================

async def main():

    drone = System()

    # --------------------------------------------------------
    # CONNECT
    # --------------------------------------------------------

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    print("Waiting for drone connection...")

    async for state in drone.core.connection_state():

        if state.is_connected:

            print("Drone connected!")

            break

    # --------------------------------------------------------
    # WAIT UNTIL ARMABLE
    # --------------------------------------------------------

    print("Waiting for PX4 to become armable...")

    async for health in drone.telemetry.health():

        if health.is_armable:

            print("PX4 is armable!")

            break

    # --------------------------------------------------------
    # ARM
    # --------------------------------------------------------

    print("Arming...")

    await drone.action.arm()

    print("Drone armed!")

    # --------------------------------------------------------
    # INITIAL VELOCITY SETPOINT
    # --------------------------------------------------------

    print("Sending initial zero velocity...")

    for _ in range(20):

        await send_velocity(drone, 0.0)

        await asyncio.sleep(0.05)

    # --------------------------------------------------------
    # START OFFBOARD
    # --------------------------------------------------------

    print("Starting Offboard mode...")

    try:

        await drone.offboard.start()

    except Exception as e:

        print("Failed to start Offboard mode:")

        print(e)

        await drone.action.disarm()

        return

    print("Offboard mode started!")

    # ========================================================
    # TAKEOFF
    # ========================================================

    print("\n========== TAKEOFF ==========")

    # Negative down velocity = upward movement
    await hold_vertical(
        drone,
        down_speed=-0.5,
        duration=TAKEOFF_TIME
    )

    # ========================================================
    # FORWARD VELOCITY TEST
    # ========================================================

    print("\n================================")
    print("      VARIABLE VELOCITY TEST")
    print("================================")

    current_speed = 0.0

    # --------------------------------------------------------
    # 0.0 -> 0.2
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.2,
        RAMP_TIME
    )

    current_speed = 0.2

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.2 -> 0.4
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.4,
        RAMP_TIME
    )

    current_speed = 0.4

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.4 -> 0.6
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.6,
        RAMP_TIME
    )

    current_speed = 0.6

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.6 -> 0.8
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.8,
        RAMP_TIME
    )

    current_speed = 0.8

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # ========================================================
    # DECELERATION
    # ========================================================

    # --------------------------------------------------------
    # 0.8 -> 0.6
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.6,
        RAMP_TIME
    )

    current_speed = 0.6

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.6 -> 0.4
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.4,
        RAMP_TIME
    )

    current_speed = 0.4

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.4 -> 0.2
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.2,
        RAMP_TIME
    )

    current_speed = 0.2

    await hold_velocity(
        drone,
        current_speed,
        HOLD_TIME
    )

    # --------------------------------------------------------
    # 0.2 -> 0.0
    # --------------------------------------------------------

    await ramp_velocity(
        drone,
        current_speed,
        0.0,
        RAMP_TIME
    )

    current_speed = 0.0

    # ========================================================
    # STOP
    # ========================================================

    print("\n========== STOP ==========")

    await hold_velocity(
        drone,
        0.0,
        STOP_TIME
    )

    # ========================================================
    # STOP OFFBOARD
    # ========================================================

    print("Stopping Offboard mode...")

    await drone.offboard.stop()

    # ========================================================
    # LAND
    # ========================================================

    print("Landing...")

    await drone.action.land()

    print("Land command sent.")

    # Give PX4 time to land
    await asyncio.sleep(8)

    print("\n================================")
    print("       TEST COMPLETE")
    print("================================")


# ============================================================
# VERTICAL MOVEMENT
# ============================================================

async def hold_vertical(drone, down_speed, duration):

    print(
        f"Vertical velocity: "
        f"{down_speed:.2f} m/s"
    )

    update_rate = 20.0
    dt = 1.0 / update_rate

    steps = int(duration * update_rate)

    for _ in range(steps):

        await send_velocity(
            drone,
            forward=0.0,
            right=0.0,
            down=down_speed
        )

        await asyncio.sleep(dt)


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    asyncio.run(main())
