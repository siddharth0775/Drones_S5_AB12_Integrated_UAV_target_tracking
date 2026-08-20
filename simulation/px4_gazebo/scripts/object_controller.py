import sys
import time
import subprocess
import termios
import tty
import select


# ============================================================
# CONFIGURATION
# ============================================================

WORLD = "default"
TARGET_NAME = "tracking_target"

# Starting position
x = 2.0
y = 0.0
z = 0.5

# Movement per key press
STEP_X = 0.20
STEP_Y = 0.20
STEP_Z = 0.15


# ============================================================
# GAZEBO SET POSE
# ============================================================

def set_target_pose(x, y, z):

    request = (
        f'name: "{TARGET_NAME}" '
        f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}} '
        f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    )

    command = [
        "gz",
        "service",
        "-s",
        f"/world/{WORLD}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "200",
        "--req",
        request
    ]

    try:

        result = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=0.5
        )

        if result.returncode != 0:

            print(
                "\nGazebo error:",
                result.stderr.strip()
            )

            return False

        return True

    except subprocess.TimeoutExpired:

        print("\nGazebo service timeout.")

        return False

    except Exception as e:

        print("\nController error:", e)

        return False


# ============================================================
# KEYBOARD
# ============================================================

def key_pressed():

    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        0
    )

    return bool(readable)


def get_key():

    return sys.stdin.read(1)


# ============================================================
# MAIN
# ============================================================

def main():

    global x, y, z

    print("=" * 50)
    print("        KEYBOARD TARGET CONTROLLER")
    print("=" * 50)

    print()
    print("TARGET:", TARGET_NAME)
    print()
    print("W / S : X forward / backward")
    print("A / D : Y left / right")
    print("R / F : Z up / down")
    print()
    print("SPACE : return target to starting position")
    print("Q     : quit")
    print()
    print("Step sizes:")
    print("X:", STEP_X, "m")
    print("Y:", STEP_Y, "m")
    print("Z:", STEP_Z, "m")
    print()

    # --------------------------------------------------------
    # TEST INITIAL POSITION
    # --------------------------------------------------------

    print("Setting initial target position...")

    if set_target_pose(x, y, z):

        print(
            f"Target position: "
            f"X={x:.2f}, "
            f"Y={y:.2f}, "
            f"Z={z:.2f}"
        )

    else:

        print(
            "\nCould not communicate with Gazebo."
        )

        return

    print()
    print("CONTROLLER ACTIVE")
    print("Click this terminal before pressing keys.")
    print()

    # --------------------------------------------------------
    # PUT TERMINAL INTO RAW MODE
    # --------------------------------------------------------

    old_settings = termios.tcgetattr(sys.stdin)

    tty.setcbreak(sys.stdin.fileno())

    try:

        while True:

            if not key_pressed():

                time.sleep(0.02)
                continue

            key = get_key().lower()

            # =================================================
            # FORWARD
            # =================================================

            if key == "w":

                x += STEP_X

            # =================================================
            # BACKWARD
            # =================================================

            elif key == "s":

                x -= STEP_X

            # =================================================
            # LEFT
            # =================================================

            elif key == "a":

                y += STEP_Y

            # =================================================
            # RIGHT
            # =================================================

            elif key == "d":

                y -= STEP_Y

            # =================================================
            # UP
            # =================================================

            elif key == "r":

                z += STEP_Z

            # =================================================
            # DOWN
            # =================================================

            elif key == "f":

                z -= STEP_Z

                # Don't let sphere go underground
                z = max(z, 0.15)

            # =================================================
            # RESET
            # =================================================

            elif key == " ":

                x = 2.0
                y = 0.0
                z = 0.5

            # =================================================
            # QUIT
            # =================================================

            elif key == "q":

                print("\nStopping controller.")

                break

            else:

                continue

            # ------------------------------------------------
            # SEND NEW POSITION
            # ------------------------------------------------

            success = set_target_pose(
                x,
                y,
                z
            )

            if success:

                print(
                    f"\rTarget: "
                    f"X={x:+.2f} "
                    f"Y={y:+.2f} "
                    f"Z={z:+.2f}       ",
                    end="",
                    flush=True
                )

    finally:

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_settings
        )

        print("\nController stopped.")


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()
