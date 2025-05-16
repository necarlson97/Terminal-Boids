from collections import defaultdict
import logging
import numpy as np
import math
from hawk import Hawk

COHESION_RADIUS = 10
ALIGNMENT_RADIUS = COHESION_RADIUS
SEPARATION_RADIUS = int(COHESION_RADIUS / 2)

COHESION_WEIGHT = 0.02
ALIGNMENT_WEIGHT = 0.03
SEPARATION_WEIGHT = 0.02

WALL_REPEL_RADIUS = 10
WALL_REPEL_WEIGHT = 0.2

HAWK_RADIUS = 10
HAWK_WEIGHT = 1.5

MAX_SPEED = 2.0
MIN_SPEED = 0.5

# Use spatial bins to optimize - for each boid,
# only check the area of the grid I'm actually in
# (and the grid's neighbors)
GRID_SIZE = COHESION_RADIUS  # Size of each square bin in pixels

class BoidSimulation:
    def __init__(self, width, height, num_boids):
        self.width = width
        self.height = height
        self.num_boids = num_boids

        self.positions = np.random.rand(num_boids, 2) * [width, height]
        self.velocities = (np.random.rand(num_boids, 2) - 0.5) * 4

        # TODO for now, one hawk
        # TODO place them more intelligently based on count
        self.hawks = [
            Hawk(width/2, height/2, GRID_SIZE),
            # Hawk(10, 10, GRID_SIZE),
            # Hawk(width-10, height-10, GRID_SIZE),
        ]

    def update(self):
        # Update all boids

        # Create dicts for spatial bins
        bins = defaultdict(list)
        for i in range(self.num_boids):
            cx = int(self.positions[i, 0] // GRID_SIZE)
            cy = int(self.positions[i, 1] // GRID_SIZE)
            bins[(cx, cy)].append(i)

        new_velocities = np.zeros_like(self.velocities)

        # Update each boid
        for i in range(self.num_boids):
            new_velocities[i] = self.update_boid(i, bins)

        # Update hawks based on all boid positions
        for hawk in self.hawks:
            hawk.update(self.positions, self.width, self.height)

        self.positions += new_velocities
        self.velocities = new_velocities
        self.positions[:, 0] %= self.width
        self.positions[:, 1] %= self.height

    def update_boid(self, i, bins):
        # Perform updates for a single boid (index)
        nearby = self.get_nearby_boids(i, bins)
        if not nearby:
            return self.velocities[i]

        new_vel = self.apply_rules(i, nearby)

        # Clamp speed
        speed = np.linalg.norm(new_vel)
        if speed > MAX_SPEED:
            new_vel *= MAX_SPEED / speed
        elif speed < MIN_SPEED:
            new_vel *= MIN_SPEED / (speed + 1e-5)

        return new_vel

    def get_nearby_boids(self, i, bins):
        # For this boid (index), get all the
        # nearby boids that can affect it's rules
        # (using the spatial bins)
        pos_i = self.positions[i]
        cx = int(pos_i[0] // GRID_SIZE)
        cy = int(pos_i[1] // GRID_SIZE)

        nearby_indices = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                cell = (cx + dx, cy + dy)
                nearby_indices.extend(bins.get(cell, []))

        if i in nearby_indices:
            nearby_indices.remove(i)

        return nearby_indices

    def apply_rules(self, i, nearby_indices):
        # For a single boid (by index),
        # apply the cohesion / separation / etc rules
        pos_i = self.positions[i]
        vel_i = self.velocities[i]

        offset = self.positions[nearby_indices] - pos_i
        distances = np.linalg.norm(offset, axis=1)
        new_vel = np.copy(vel_i)

        # Cohesion
        mask = distances < COHESION_RADIUS
        if np.any(mask):
            avg_pos = np.mean(self.positions[nearby_indices][mask], axis=0)
            new_vel += (avg_pos - pos_i) * COHESION_WEIGHT

        # Alignment
        mask = distances < ALIGNMENT_RADIUS
        if np.any(mask):
            avg_vel = np.mean(self.velocities[nearby_indices][mask], axis=0)
            new_vel += (avg_vel - vel_i) * ALIGNMENT_WEIGHT

        # Separation
        mask = distances < SEPARATION_RADIUS
        if np.any(mask):
            repulse = -np.sum(
                offset[mask] / (distances[mask][:, None] + 1e-5), axis=0)
            new_vel += repulse * SEPARATION_WEIGHT

        # Repelled by hawks
        for hawk in self.hawks:
            offset = self.positions[i] - hawk.pos
            dist = np.linalg.norm(offset)
            if dist < HAWK_RADIUS:
                repel = offset / (dist + 1e-5)
                new_vel += repel * HAWK_WEIGHT

        # Keep the boids in the central circle
        cx, cy = self.width / 2, self.height / 2
        x, y = self.positions[i]
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy)

        circle_radius = min(self.width, self.height) / 2
        circle_buffer = WALL_REPEL_RADIUS  # same value, reused
        edge_radius = circle_radius - circle_buffer

        if dist > edge_radius:
            excess = dist - edge_radius
            repel_strength = (excess / circle_buffer) * WALL_REPEL_WEIGHT
            direction = np.array([-dx, -dy]) / (dist + 1e-5)
            new_vel += direction * repel_strength

        return new_vel
