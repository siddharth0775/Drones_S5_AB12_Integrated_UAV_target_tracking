"""
run_tracking.py

Main entry point for the MuJoCo UAV closed-loop target tracking simulation.

Supports:
  --trajectory    : Select from 10 trajectory types
  --duration      : Simulation length (seconds)
  --headless      : Disable MuJoCo interactive viewer
  --no-hud        : Disable OpenCV drone-camera window
  --keyboard      : Enable WASD/RF manual target control (mirrors PyBullet)
  --test-all      : Run all 10 trajectories in automated batch mode

Usage examples
--------------
  python run_tracking.py --trajectory circle --duration 25
  python run_tracking.py --trajectory wander --duration 30 --keyboard
  python run_tracking.py --trajectory helix  --duration 20 --headless
  python run_tracking.py --trajectory zigzag --duration 20 --no-hud
  python run_tracking.py --test-all --headless --duration 15

Trajectory Options
------------------
  Original (matching PyBullet):
    stationary, line, circle, figure8, waypoint, wander

  New MuJoCo-native:
    sine, helix, lissajous, zigzag
"""

import argparse
import sys
import os

# Add local directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.simulation import UAVTrackingSimulation
from configs.config import SimConfig

# All supported trajectories (10 total)
ALL_TRAJECTORIES = [
    "stationary", "line", "circle", "sine",
    "figure8", "waypoint", "helix", "lissajous", "zigzag", "wander",
]


def run_single_experiment(
    trajectory_type: str,
    duration: float,
    headless: bool,
    show_hud: bool,
    keyboard_mode: bool,
):
    print("=" * 70)
    print(f"  MUJOCO UAV CLOSED-LOOP TRACKING  [{trajectory_type.upper()}]")
    if keyboard_mode:
        print("  KEYBOARD MODE: W/S=fwd/bwd  A/D=left/right  R/F=up/dn  ESC=quit")
        print("  (Click the OpenCV window to capture keyboard input)")
    print("=" * 70)

    sim = UAVTrackingSimulation(
        trajectory_type=trajectory_type,
        config=SimConfig,
    )

    metrics = sim.run(
        duration=duration,
        headless=headless,
        print_freq=1.0,
        show_hud=show_hud,
        keyboard_mode=keyboard_mode,
    )

    print("\n" + "=" * 70)
    print("  EVALUATION METRICS REPORT")
    print("=" * 70)
    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"  {key:<30}: {value:8.3f}")
        else:
            print(f"  {key:<30}: {value}")
    print("=" * 70)
    return metrics


def run_all_trajectories_test(
    duration: float = 15.0,
    headless: bool = True,
    show_hud: bool = False,
):
    print("=" * 70)
    print(f"  AUTOMATED MULTI-TRAJECTORY TEST SUITE  ({len(ALL_TRAJECTORIES)} TRAJECTORIES)")
    print("=" * 70)

    results = {}
    for traj in ALL_TRAJECTORIES:
        print(f"\n>>> Running: {traj.upper()} ...")
        try:
            metrics = run_single_experiment(
                traj, duration=duration, headless=headless,
                show_hud=show_hud, keyboard_mode=False
            )
            results[traj] = metrics
        except Exception as e:
            print(f"  ERROR in '{traj}': {e}")
            results[traj] = {"error": str(e)}

    # Summary matrix
    print("\n" + "=" * 70)
    print("  SUMMARY MATRIX")
    print("=" * 70)
    print(f"{'Trajectory':<12} | {'Detect %':<10} | {'RMSE (m)':<10} | "
          f"{'DistErr (m)':<13} | {'Losses':<8}")
    print("-" * 70)
    for traj, m in results.items():
        if "error" in m:
            print(f"{traj:<12} | FAILED: {m['error']}")
        else:
            print(
                f"{traj:<12} | {m.get('detection_rate_pct', 0):9.1f}% | "
                f"{m.get('rmse_pos_error_m', 0):10.3f} | "
                f"{m.get('mean_dist_error_m', 0):13.3f} | "
                f"{m.get('target_loss_count', 0):<8}"
            )
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="MuJoCo UAV Closed-Loop Target Tracking Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--trajectory", type=str, default="circle",
        choices=ALL_TRAJECTORIES,
        help="Target trajectory type (default: circle)",
    )
    parser.add_argument(
        "--duration", type=float, default=20.0,
        help="Simulation duration in seconds (default: 20)",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Disable MuJoCo interactive viewer window",
    )
    parser.add_argument(
        "--no-hud", action="store_true",
        help="Disable the OpenCV drone-camera HUD window",
    )
    parser.add_argument(
        "--keyboard", action="store_true",
        help="Enable WASD/RF keyboard control of target (mirrors PyBullet mode)",
    )
    parser.add_argument(
        "--test-all", action="store_true",
        help="Run all 10 trajectories in automated batch evaluation",
    )

    args = parser.parse_args()

    show_hud = not args.no_hud

    if args.test_all:
        run_all_trajectories_test(
            duration=args.duration,
            headless=args.headless,
            show_hud=show_hud,
        )
    else:
        run_single_experiment(
            trajectory_type=args.trajectory,
            duration=args.duration,
            headless=args.headless,
            show_hud=show_hud,
            keyboard_mode=args.keyboard,
        )


if __name__ == "__main__":
    main()
