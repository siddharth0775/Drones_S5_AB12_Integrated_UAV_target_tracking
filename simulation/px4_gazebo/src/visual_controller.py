import math


# ============================================================
# VISUAL CONTROLLER CONFIGURATION
# ============================================================

# Proportional gains
K_X = 0.5
K_Y = 0.5

# Maximum velocity command
MAX_VELOCITY = 1.0

# Small deadband to prevent constant tiny movements
DEADBAND_DEG = 1.0


# ============================================================
# LIMIT VELOCITY
# ============================================================

def clamp(value, minimum, maximum):

    return max(minimum, min(value, maximum))


# ============================================================
# VISUAL P CONTROLLER
# ============================================================

def calculate_velocity(angle_x_deg, angle_y_deg):

    # --------------------------------------------------------
    # Deadband
    # --------------------------------------------------------

    if abs(angle_x_deg) < DEADBAND_DEG:
        angle_x_deg = 0.0

    if abs(angle_y_deg) < DEADBAND_DEG:
        angle_y_deg = 0.0

    # --------------------------------------------------------
    # Convert degrees → radians
    # --------------------------------------------------------

    angle_x = math.radians(angle_x_deg)
    angle_y = math.radians(angle_y_deg)

    # --------------------------------------------------------
    # Proportional controller
    # --------------------------------------------------------

    vx = K_X * angle_x
    vz = -K_Y * angle_y

    # --------------------------------------------------------
    # Limit velocity
    # --------------------------------------------------------

    vx = clamp(
        vx,
        -MAX_VELOCITY,
        MAX_VELOCITY
    )

    vz = clamp(
        vz,
        -MAX_VELOCITY,
        MAX_VELOCITY
    )

    # No lateral movement yet
    vy = 0.0

    return vx, vy, vz


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    test_angles = [
        (0, 0),
        (5, 0),
        (-5, 0),
        (0, 5),
        (0, -5),
        (15, 10),
        (-15, -10)
    ]

    print("Visual Controller Test")
    print("======================")

    for angle_x, angle_y in test_angles:

        vx, vy, vz = calculate_velocity(
            angle_x,
            angle_y
        )

        print(
            f"Angle=({angle_x:+6.2f}, "
            f"{angle_y:+6.2f})° "
            f"→ "
            f"Velocity=({vx:+.3f}, "
            f"{vy:+.3f}, "
            f"{vz:+.3f}) m/s"
        )
