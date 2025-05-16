import math
import numpy as np

SPEED = 1.5
SMOOTHING = 0.15  # Lower is more smoothing

class Hawk:
    def __init__(self, x, y, grid_size):
        self.grid_size = grid_size
        self.pos = np.array([x, y], dtype=np.float32)
        self.vel = np.random.uniform(-1, 1, size=2)

    def update(self, boid_positions, width, height):
        if len(boid_positions) == 0:
            return

        # Find the closest boid
        offsets = boid_positions - self.pos
        dists = np.linalg.norm(offsets, axis=1)

        nearest_index = np.argmin(dists)
        nearest_dist = dists[nearest_index]

        if nearest_dist > 200:  # Optional max chase distance
            return  # Too far, don't adjust

        target = boid_positions[nearest_index]
        direction = target - self.pos
        norm = np.linalg.norm(direction)
        if norm > 0:
            desired_velocity = (direction / norm) * SPEED
            self.vel = (1 - SMOOTHING) * self.vel + SMOOTHING * desired_velocity

        # Apply position update
        self.pos += self.vel
        self.pos[0] = np.clip(self.pos[0], 0, width)
        self.pos[1] = np.clip(self.pos[1], 0, height)

    def direction_char(self):
        dx, dy = self.vel
        angle = math.atan2(dy, dx)
        angle_deg = (math.degrees(angle) + 360) % 360
        if 337.5 <= angle_deg or angle_deg < 22.5:
            return '➡'
        elif 22.5 <= angle_deg < 67.5:
            return '⬊'
        elif 67.5 <= angle_deg < 112.5:
            return '⬇'
        elif 112.5 <= angle_deg < 157.5:
            return '⬋'
        elif 157.5 <= angle_deg < 202.5:
            return '⬅'
        elif 202.5 <= angle_deg < 247.5:
            return '⬉'
        elif 247.5 <= angle_deg < 292.5:
            return '⬆'
        elif 292.5 <= angle_deg < 337.5:
            return '⬈'
        return '?'
