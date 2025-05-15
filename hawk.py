import math
import numpy as np

SPEED = 1.0
SMOOTHING = 0.15  # Lower is more smoothing

class Hawk:
    def __init__(self, x, y, grid_size):
        self.grid_size = grid_size
        self.pos = np.array([x, y], dtype=np.float32)
        self.vel = np.random.uniform(-1, 1, size=2)

    def update(self, boid_positions, width, height):
        sector_weights = [0.0] * 8
        sector_vectors = [
            np.array([0, 1]),   # N
            np.array([1, 1]),   # NE
            np.array([1, 0]),    # E
            np.array([1, -1]),    # SE
            np.array([0, -1]),    # S
            np.array([-1, -1]),   # SW
            np.array([-1, 0]),   # W
            np.array([-1, 1])   # NW
        ]

        for pos in boid_positions:
            offset = pos - self.pos
            dist = np.linalg.norm(offset)
            if dist == 0 or dist > 200:  # Optional max influence range
                continue

            angle = math.atan2(offset[0], offset[1])
            angle_deg = (math.degrees(angle) + 360) % 360
            sector = int(((angle_deg + 22.5) % 360) // 45)

            # Inverse-square weighting
            weight = 1.0 / (dist**2 + 1e-5)
            sector_weights[sector] += weight

        if max(sector_weights) == 0:
            return  # No boids to track

        best_sector = np.argmax(sector_weights)
        target_direction = sector_vectors[best_sector]

        # Smooth the hawk’s velocity toward the target
        desired_velocity = target_direction * SPEED
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
