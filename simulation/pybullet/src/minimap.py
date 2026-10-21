import csv
from pathlib import Path

import matplotlib.pyplot as plt


class DroneMinimap:
    """Live top-down view of drone and target positions in world coordinates."""

    def __init__(self, results_dir, redraw_every=3):
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.redraw_every = max(1, int(redraw_every))
        self.frame_count = 0
        self.closed = False

        self.drone_history = []
        self.target_history = []
        self.time_history = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 7))

        self.ax.set_title("Live PyBullet Drone Minimap")
        self.ax.set_xlabel("World X (m)")
        self.ax.set_ylabel("World Y (m)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.3)

        self.drone_line, = self.ax.plot(
            [], [], color="tab:orange", label="Drone path"
        )
        self.target_line, = self.ax.plot(
            [], [], color="tab:blue", label="Target path"
        )

        self.drone_marker, = self.ax.plot(
            [], [], "o", color="tab:orange", markersize=8,
            label="Drone"
        )
        self.target_marker, = self.ax.plot(
            [], [], "o", color="tab:green", markersize=8,
            label="Target"
        )

        self.ax.legend(loc="upper right")
        self.ax.margins(0.15)

        self.fig.canvas.mpl_connect(
            "close_event", self._on_close
        )
        plt.show(block=False)
        plt.pause(0.1)

    def _on_close(self, event):
        self.closed = True

    def update(self, elapsed_time, drone_position, target_position):
        """Record current positions and refresh the live plot."""

        if self.closed:
            return

        drone_x, drone_y = map(float, drone_position[:2])
        target_x, target_y = map(float, target_position[:2])

        self.time_history.append(float(elapsed_time))
        self.drone_history.append((drone_x, drone_y))
        self.target_history.append((target_x, target_y))

        self.frame_count += 1

        # Keep recording every sample, but redraw less frequently.
        if self.frame_count % self.redraw_every != 0:
            return

        self.drone_line.set_data(
            [point[0] for point in self.drone_history],
            [point[1] for point in self.drone_history],
        )
        self.target_line.set_data(
            [point[0] for point in self.target_history],
            [point[1] for point in self.target_history],
        )

        self.drone_marker.set_data([drone_x], [drone_y])
        self.target_marker.set_data([target_x], [target_y])

        # Expand the view automatically as either object moves.
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_aspect("equal", adjustable="box")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def save(self):
        """Save the final trajectory plot and all recorded positions."""

        csv_path = self.results_dir / "minimap_trajectory.csv"
        png_path = self.results_dir / "minimap_trajectory.png"

        with csv_path.open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow([
                "time",
                "drone_x", "drone_y",
                "target_x", "target_y",
            ])

            for t, drone, target in zip(
                self.time_history,
                self.drone_history,
                self.target_history,
            ):
                writer.writerow([t, *drone, *target])

        # Ensure the final recorded sample is visible in the saved plot.
        if self.drone_history:
            self.drone_line.set_data(
                [p[0] for p in self.drone_history],
                [p[1] for p in self.drone_history],
            )
            self.target_line.set_data(
                [p[0] for p in self.target_history],
                [p[1] for p in self.target_history],
            )
            self.drone_marker.set_data(
                [self.drone_history[-1][0]],
                [self.drone_history[-1][1]],
            )
            self.target_marker.set_data(
                [self.target_history[-1][0]],
                [self.target_history[-1][1]],
            )
            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.set_aspect("equal", adjustable="box")

        self.fig.savefig(png_path, dpi=180, bbox_inches="tight")

        print(f"Minimap trajectory CSV: {csv_path}")
        print(f"Minimap trajectory PNG: {png_path}")

    def close(self):
        self.save()

        if not self.closed:
            plt.close(self.fig)
