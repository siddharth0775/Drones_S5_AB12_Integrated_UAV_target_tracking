"""
track_demo.py

PyBullet object-tracking drone simulation.

Controls:

W / S : target forward / backward
A / D : target left / right
R / F : target up / down
ESC   : quit

The target keyboard is handled by PyBullet.
OpenCV is used only for displaying the drone camera.
"""

import numpy as np
import cv2
import pybullet as p

import csv
import os
import time

from camera_tracker import (
    ColorBlobTracker,
    TrackingController
)

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl


# ================================================================
# SIMULATION PARAMETERS
# ================================================================

SIM_FREQ = 240
CTRL_FREQ = 48

FRAME_W = 320
FRAME_H = 240

TARGET_MOVE_SPEED = 1.5

# ================================================================
# TRACKING DATA LOGGER
# ================================================================

RESULTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "results"
)

os.makedirs(
    RESULTS_DIR,
    exist_ok=True
)

LOG_FILE = os.path.join(
    RESULTS_DIR,
    "tracking_log.csv"
)

# ================================================================
# CAMERA
# ================================================================

def get_drone_camera_image(
    env,
    nth_drone=0,
    img_w=FRAME_W,
    img_h=FRAME_H,
    fov=110
):

    state = env._getDroneStateVector(nth_drone)

    pos = state[0:3]
    quat = state[3:7]

    rot = np.array(
        p.getMatrixFromQuaternion(quat)
    ).reshape(3, 3)

    # Drone local axes
    forward = rot.dot(
        np.array([1.0, 0.0, 0.0])
    )

    up = rot.dot(
        np.array([0.0, 0.0, 1.0])
    )

    cam_pos = (
        pos
        + rot.dot(
            np.array([0.0, 0.0, 0.02])
        )
    )

    cam_target = (
        cam_pos
        + forward * 1.0
    )

    view_matrix = p.computeViewMatrix(
        cam_pos,
        cam_target,
        up
    )

    proj_matrix = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=img_w / img_h,
        nearVal=0.05,
        farVal=20.0
    )

    _, _, rgb_img, _, _ = p.getCameraImage(
        img_w,
        img_h,
        view_matrix,
        proj_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=env.CLIENT,
    )

    rgb = np.reshape(
        rgb_img,
        (img_h, img_w, 4)
    )[:, :, :3].astype(np.uint8)

    return rgb


# ================================================================
# TARGET
# ================================================================

def spawn_target_sphere(
    env,
    radius=0.08,
    rgba=(0.0, 1.0, 0.0, 1.0)
):

    visual_id = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=rgba,
        physicsClientId=env.CLIENT
    )

    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=[0, 0, 1.0],
        physicsClientId=env.CLIENT
    )

    return body_id


def move_target_sphere(
    env,
    body_id,
    position
):

    p.resetBasePositionAndOrientation(
        body_id,
        position,
        [0, 0, 0, 1],
        physicsClientId=env.CLIENT
    )


# ================================================================
# KEYBOARD
# ================================================================

def get_target_keyboard_input():

    keys = p.getKeyboardEvents()

    move = np.array(
        [0.0, 0.0, 0.0]
    )

    # PyBullet keyboard flags
    pressed = p.KEY_IS_DOWN

    if ord('w') in keys and keys[ord('w')] & pressed:
        move[0] += 1.0

    if ord('s') in keys and keys[ord('s')] & pressed:
        move[0] -= 1.0

    if ord('a') in keys and keys[ord('a')] & pressed:
        move[1] -= 1.0

    if ord('d') in keys and keys[ord('d')] & pressed:
        move[1] += 1.0

    if ord('r') in keys and keys[ord('r')] & pressed:
        move[2] += 1.0

    if ord('f') in keys and keys[ord('f')] & pressed:
        move[2] -= 1.0

    return move, False


# ================================================================
# MAIN
# ================================================================

def main():
    
    # ============================================================
    # START TRACKING DATA LOG
    # ============================================================

    log_file = open(
        LOG_FILE,
        "w",
        newline=""
    )

    logger = csv.writer(log_file)

    logger.writerow([
        "time",

        # Drone ground-truth position
        "drone_x",
        "drone_y",
        "drone_z",

        # Target ground-truth position
        "target_x",
        "target_y",
        "target_z",

        # 3D tracking error
        "error_x",
        "error_y",
        "error_z",
        "position_error",

        # Camera target position
        "pixel_x",
        "pixel_y",
        "target_radius_px",

        # Camera center
        "image_center_x",
        "image_center_y",

        # Pixel tracking error
        "pixel_error_x",
        "pixel_error_y",

        # Controller commands
        "vx_command",
        "vz_command",
        "yaw_rate_command",
        "commanded_yaw",

        # Tracking state
        "tracking_state"
    ])

    experiment_start = time.time()

    # ============================================================
    # INITIAL TARGET POSITION
    # ============================================================

    manual_target_pos = np.array(
        [0.0, 0.0, 1.0],
        dtype=float
    )

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,

        initial_xyzs=np.array([
            [0.0, 0.0, 1.0]
        ]),

        physics=Physics.PYB,

        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ,

        gui=True,
    )

    ctrl = DSLPIDControl(
        drone_model=DroneModel.CF2X
    )

    tracker = ColorBlobTracker()

    tracking_controller = TrackingController(
        target_radius_px=30
    )

    obs, info = env.reset()

    target_body_id = spawn_target_sphere(
        env
    )

    action = np.zeros(
        (1, 4)
    )

    commanded_pos = np.array(
        obs[0][0:3],
        dtype=float
    )

    dt = 1.0 / CTRL_FREQ

    commanded_yaw = 0.0

    step = 0

    print()
    print("=" * 60)
    print("PYBULLET OBJECT TRACKING")
    print("=" * 60)
    print()
    print("TARGET CONTROLS:")
    print("W / S : forward / backward")
    print("A / D : left / right")
    print("R / F : up / down")
    print()
    print("ESC : quit")
    print()
    print("Click the PyBullet window before using target controls.")
    print()

    try:

        while True:

            # ====================================================
            # 1. TARGET KEYBOARD
            # ====================================================

            move, _ = (
                get_target_keyboard_input()
            )

            manual_target_pos += (
                move
                * TARGET_MOVE_SPEED
                * dt
            )

            manual_target_pos[2] = max(
                manual_target_pos[2],
                0.15
            )

            move_target_sphere(
                env,
                target_body_id,
                manual_target_pos
            )

            # ====================================================
            # 2. CAMERA
            # ====================================================

            rgb = get_drone_camera_image(
                env
            )

            # ====================================================
            # 3. TARGET DETECTION
            # ====================================================

            cx, cy, radius_px, found = (
                tracker.find_target(rgb)
            )

            # ====================================================
            # 4. TRACKING CONTROLLER
            # ====================================================

            cmd = tracking_controller.update(
                cx if cx is not None else 0,
                cy if cy is not None else 0,
                radius_px if radius_px is not None else 0,

                FRAME_W,
                FRAME_H,

                found,
                dt
            )

            # ====================================================
            # 5. CURRENT DRONE STATE
            # ====================================================

            state = obs[0]

            cur_pos = state[0:3]

            cur_quat = state[3:7]

            # ====================================================
            # 6. BODY → WORLD
            # ====================================================

            rot = np.array(
                p.getMatrixFromQuaternion(
                    cur_quat
                )
            ).reshape(3, 3)

            forward = rot.dot(
                np.array([1.0, 0.0, 0.0])
            )

            # Current controller only uses:
            #
            # vx = body forward/backward
            # vz = world vertical
            #
            world_vel = (
                forward * cmd["vx"]
                + np.array(
                    [0.0, 0.0, cmd["vz"]]
                )
            )

            # ====================================================
            # 7. POSITION SETPOINT
            # ====================================================

            target_step_pos = (
                commanded_pos
                + world_vel * dt
            )

            # Prevent runaway setpoints.

            lead_vec = (
                target_step_pos
                - cur_pos
            )

            max_lead = 0.35

            lead_dist = np.linalg.norm(
                lead_vec
            )

            if lead_dist > max_lead:

                target_step_pos = (
                    cur_pos
                    + lead_vec
                    / lead_dist
                    * max_lead
                )

            commanded_pos = target_step_pos

            # ====================================================
            # 8. YAW SETPOINT
            # ====================================================

            commanded_yaw += (
                cmd["yaw_rate"]
                * dt
            )

            # Keep yaw numerically bounded.

            commanded_yaw = (
                (commanded_yaw + np.pi)
                % (2.0 * np.pi)
                - np.pi
            )

            # ====================================================
            # 9. DRONE CONTROLLER
            # ====================================================

            action[0, :], _, _ = (
                ctrl.computeControlFromState(
                    control_timestep=dt,

                    state=state,

                    target_pos=target_step_pos,

                    target_rpy=np.array([
                        0.0,
                        0.0,
                        commanded_yaw
                    ])
                )
            )

            # ====================================================
            # 10. PHYSICS STEP
            # ====================================================
            
            #Make sure PyBullet is still connected before stepping.
            if not p.isConnected(env.CLIENT):
                print("PyBullet connection closed.")
                break
            
            obs, reward, terminated, truncated, info = (
                env.step(action)
            )
            
            # ====================================================
            # 10.5. LOG TRACKING DATA
            # ====================================================
            
            # Get the NEW drone position after the physics step.
            new_state = obs[0]
            
            new_drone_pos = new_state[0:3]
            
            # ----------------------------------------------------
            # 3D POSITION ERROR
            # ----------------------------------------------------
            
            position_error_vector = (
                manual_target_pos
                - new_drone_pos
            )
            
            position_error = np.linalg.norm(
                position_error_vector
            )
            
            # ----------------------------------------------------
            # CAMERA PIXEL ERROR
            # ----------------------------------------------------
            
            image_center_x = FRAME_W / 2.0
            image_center_y = FRAME_H / 2.0
            
            if found:
            
                pixel_error_x = (
                    cx - image_center_x
                )
            
                pixel_error_y = (
                    cy - image_center_y
                )
            
            else:
            
                pixel_error_x = None
                pixel_error_y = None
            
            # ----------------------------------------------------
            # TRACKING STATE
            # ----------------------------------------------------
            
            tracking_state = (
                str(tracking_controller.state)
            )
            
            # ----------------------------------------------------
            # WRITE DATA
            # ----------------------------------------------------
            
            logger.writerow([
            
                time.time() - experiment_start,
            
                # Drone position
                new_drone_pos[0],
                new_drone_pos[1],
                new_drone_pos[2],
            
                # Target position
                manual_target_pos[0],
                manual_target_pos[1],
                manual_target_pos[2],
            
                # Position error
                position_error_vector[0],
                position_error_vector[1],
                position_error_vector[2],
                position_error,
            
                # Camera detection
                cx if found else None,
                cy if found else None,
                radius_px if found else None,
            
                # Image center
                image_center_x,
                image_center_y,
            
                # Pixel error
                pixel_error_x,
                pixel_error_y,
            
                # Controller commands
                cmd["vx"],
                cmd["vz"],
                cmd["yaw_rate"],
                commanded_yaw,
            
                # State
                tracking_state
            ])
            
            # Make sure the data is physically written to disk.
            log_file.flush()

            # ====================================================
            # 11. CAMERA HUD
            # ====================================================

            hud = rgb.copy()

            center_x = FRAME_W // 2
            center_y = FRAME_H // 2

            # Crosshair

            cv2.drawMarker(
                hud,
                (center_x, center_y),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                15,
                1
            )

            if found:
                
                error_x = (cx - FRAME_W / 2) / (FRAME_W / 2)
                error_y = (FRAME_H / 2 - cy) / (FRAME_H / 2)
                
                cv2.putText(
                    hud,
                    f"ex: {error_x:+.2f}",
                    (180, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1
                )
                
                cv2.putText(
                    hud,
                    f"ey: {error_y:+.2f}",
                    (180, 38),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1
                )

                cv2.circle(
                    hud,
                    (
                        int(cx),
                        int(cy)
                    ),
                    int(radius_px),
                    (0, 255, 0),
                    2
                )

                cv2.line(
                    hud,
                    (center_x, center_y),
                    (int(cx), int(cy)),
                    (255, 255, 0),
                    1
                )

                cv2.putText(
                    hud,
                    "TRACKING",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

            else:

                cv2.putText(
                    hud,
                    "SEARCHING",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            # Controller information

            cv2.putText(
                hud,
                f"vx: {cmd['vx']:.2f}",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1
            )

            cv2.putText(
                hud,
                f"vz: {cmd['vz']:.2f}",
                (10, 63),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1
            )

            cv2.putText(
                hud,
                f"yaw: {cmd['yaw_rate']:.2f}",
                (10, 81),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1
            )

            cv2.putText(
                hud,
                "WASD/RF: move target | ESC: quit",
                (10, FRAME_H - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1
            )

            # ====================================================
            # 12. DISPLAY
            # ====================================================

            cv2.imshow(
                "Drone Camera",
                cv2.cvtColor(
                    hud,
                    cv2.COLOR_RGB2BGR
                )
            )

            # IMPORTANT:
            #
            # waitKey is now ONLY used to keep OpenCV's
            # window alive/repainted.
            #
            # It is NOT used to control the target.

            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  #ESC
                print("ESC pressed - stopping simulation.")
                break

            # ====================================================
            # 13. DEBUG
            # ====================================================

            step += 1

            if step % 48 == 0:

                print(
                    f"[DEBUG] "
                    f"found={found} "
                    f"cx={cx} "
                    f"cy={cy} "
                    f"radius={radius_px} "
                    f"state={tracking_controller.state} "
                    f"cmd={ {k: round(v, 3) for k, v in cmd.items()} } "
                    f"drone={np.round(cur_pos, 2)} "
                    f"target={np.round(manual_target_pos, 2)}"
                )

            # ====================================================
            # 14. EXIT IF CAMERA WINDOW CLOSED
            # ====================================================

            if (
                cv2.getWindowProperty(
                    "Drone Camera",
                    cv2.WND_PROP_VISIBLE
                ) < 1
            ):
                break

            if not p.isConnected(
                env.CLIENT
            ):
                break

    finally:
        
        # ========================================================
        # CLOSE TRACKING LOG
        # ========================================================
        
        try:
            log_file.close()
            print(
                f"\nTracking data saved to:\n{LOG_FILE}"
            )
        except Exception as e:
            print(
                f"Could not close tracking log: {e}"
            )

        # Only call env.close() if the physics server is
        # still connected. Otherwise gym-pybullet-drones
        # tries to disconnect an already disconnected server.
    
        try:
            if p.isConnected(env.CLIENT):
                env.close()
        except Exception as e:
            print(f"Simulation cleanup warning: {e}")
    
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()