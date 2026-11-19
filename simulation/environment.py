"""
2D Continuous Simulation Environment for Autonomous Obstacle Avoidance Car.
Generates customizable indoor arenas with walls and obstacle pillars.
Includes robust collision checking and verified safe spawn positioning.
"""

import math
import random
import numpy as np
from typing import List, Tuple, Dict, Optional

from simulation.vehicle import Vehicle
from simulation.sensors import SensorArray, PRESET_CARDINAL_4
from reinforcement_learning.rewards import RewardCalculator

class ObstacleCourse:
    """Helper to generate segments and boxes for arena walls and rectangular obstacles."""
    BOXES = [
        (60.0, 60.0, 40.0, 40.0),    # Pillar 1
        (200.0, 60.0, 40.0, 40.0),   # Pillar 2
        (120.0, 140.0, 60.0, 30.0),  # Central divider
        (60.0, 200.0, 40.0, 40.0),   # Pillar 3
        (200.0, 200.0, 40.0, 40.0),  # Pillar 4
    ]

    @staticmethod
    def create_arena_segments(width: float = 300.0, height: float = 300.0) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Outer boundary walls."""
        return [
            ((0, 0), (width, 0)),
            ((width, 0), (width, height)),
            ((width, height), (0, height)),
            ((0, height), (0, 0))
        ]

    @staticmethod
    def create_box_segments(x: float, y: float, w: float, h: float) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """A rectangular obstacle."""
        return [
            ((x, y), (x + w, y)),
            ((x + w, y), (x + w, y + h)),
            ((x + w, y + h), (x, y + h)),
            ((x, y + h), (x, y))
        ]

    @staticmethod
    def get_default_course(arena_w: float = 300.0, arena_h: float = 300.0) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        segments = ObstacleCourse.create_arena_segments(arena_w, arena_h)
        for bx, by, bw, bh in ObstacleCourse.BOXES:
            segments.extend(ObstacleCourse.create_box_segments(bx, by, bw, bh))
        return segments

    @staticmethod
    def is_inside_or_near_box(x: float, y: float, margin: float = 25.0) -> bool:
        """Checks if a point is within margin distance of any obstacle box."""
        for bx, by, bw, bh in ObstacleCourse.BOXES:
            if (bx - margin <= x <= bx + bw + margin) and (by - margin <= y <= by + bh + margin):
                return True
        return False


class CarArenaEnv:
    def __init__(self,
                 arena_width: float = 300.0,
                 arena_height: float = 300.0,
                 sensor_layout: Optional[List[Dict]] = None,
                 max_steps_per_episode: int = 250,
                 dt: float = 0.1):
        self.arena_w = arena_width
        self.arena_h = arena_height
        self.max_steps = max_steps_per_episode
        self.dt = dt

        self.vehicle = Vehicle()
        self.sensor_array = SensorArray(layout=sensor_layout)
        self.reward_calculator = RewardCalculator()

        self.segments = ObstacleCourse.get_default_course(self.arena_w, self.arena_h)
        self.current_step = 0
        self.is_done = False

    def reset(self, random_spawn: bool = True) -> Tuple[int, Dict]:
        """Resets the car inside the arena in a guaranteed collision-free, clear location."""
        self.current_step = 0
        self.is_done = False

        if random_spawn:
            spawned = False
            for _ in range(500):
                rx = random.uniform(35.0, self.arena_w - 35.0)
                ry = random.uniform(35.0, self.arena_h - 35.0)

                # Ensure vehicle is not inside or too close to any pillar
                if ObstacleCourse.is_inside_or_near_box(rx, ry, margin=25.0):
                    continue

                heading = random.uniform(0.0, 360.0)
                self.vehicle.reset(rx, ry, heading)

                # Verify all sensor clearances
                dists = self.sensor_array.get_readings(
                    self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.segments, add_noise=False
                )
                if min(dists) >= 30.0:
                    spawned = True
                    break

            if not spawned:
                # Safe fallback spawn at center corridor
                self.vehicle.reset(150.0, 45.0, 90.0)
        else:
            self.vehicle.reset(150.0, 45.0, 90.0)

        dists = self.sensor_array.get_readings(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.segments, add_noise=True
        )
        state_id, s_tuple = self.sensor_array.get_discrete_state(dists)

        info = {
            "distances": dists,
            "discrete_states": s_tuple,
            "x": self.vehicle.x,
            "y": self.vehicle.y,
            "heading": self.vehicle.heading,
            "step": self.current_step
        }
        return state_id, info

    def check_collision(self, distances: List[float]) -> bool:
        """Collision occurs if vehicle bumper contacts an obstacle (< 10 cm) or boundary."""
        radius = self.vehicle.get_bounding_radius()
        if (self.vehicle.x - radius < 0 or self.vehicle.x + radius > self.arena_w or
            self.vehicle.y - radius < 0 or self.vehicle.y + radius > self.arena_h):
            return True

        # Any sensor reading below 10cm represents physical contact with obstacle
        if min(distances) < 10.0:
            return True

        return False

    def step(self, action: int) -> Tuple[int, float, bool, bool, Dict]:
        """
        Executes one action step.
        Returns: (next_state_id, reward, terminated, truncated, info)
        """
        self.current_step += 1

        self.vehicle.step(action, dt=self.dt)

        distances = self.sensor_array.get_readings(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.segments, add_noise=True
        )
        state_id, s_tuple = self.sensor_array.get_discrete_state(distances)

        collision = self.check_collision(distances)
        terminated = collision
        truncated = (self.current_step >= self.max_steps)

        reward = self.reward_calculator.calculate_reward(
            action=action,
            discrete_states=s_tuple,
            continuous_distances=distances,
            collision=collision
        )

        self.is_done = terminated or truncated

        info = {
            "distances": distances,
            "discrete_states": s_tuple,
            "x": self.vehicle.x,
            "y": self.vehicle.y,
            "heading": self.vehicle.heading,
            "collision": collision,
            "step": self.current_step
        }

        return state_id, reward, terminated, truncated, info
