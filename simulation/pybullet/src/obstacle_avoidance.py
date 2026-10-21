"""Simple spherical-obstacle setpoint safety filter for the PyBullet demo."""
import numpy as np


class ObstacleAvoidance:
    """Push desired position setpoints outside inflated spherical obstacles.

    This is a first-stage demonstration filter, not a complete collision
    guarantee: it does not yet perform swept-path or velocity-based checks.
    """

    def __init__(self, safety_margin=0.45):
        self.safety_margin = float(safety_margin)
        self.obstacles = []
        self.active = False

    def add_obstacle(self, position, radius):
        position = np.asarray(position, dtype=float).reshape(3)
        radius = float(radius)
        if radius <= 0:
            raise ValueError("Obstacle radius must be positive")
        self.obstacles.append({"position": position, "radius": radius})

    def filter_setpoint(self, current_position, desired_position):
        current = np.asarray(current_position, dtype=float).reshape(3)
        desired = np.asarray(desired_position, dtype=float).reshape(3).copy()
        self.active = False

        # Iterate more than once so a correction around one obstacle is checked
        # against the remaining obstacles too.
        for _ in range(max(1, len(self.obstacles))):
            changed = False
            for obstacle in self.obstacles:
                center = obstacle["position"]
                safe_radius = obstacle["radius"] + self.safety_margin
                offset = desired - center
                distance = float(np.linalg.norm(offset))
                if distance < safe_radius:
                    if distance > 1e-8:
                        direction = offset / distance
                    else:
                        current_offset = current - center
                        current_distance = float(np.linalg.norm(current_offset))
                        direction = (
                            current_offset / current_distance
                            if current_distance > 1e-8
                            else np.array([1.0, 0.0, 0.0])
                        )
                    desired = center + direction * safe_radius
                    self.active = True
                    changed = True
            if not changed:
                break

        return desired
