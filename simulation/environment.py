"""
2D Continuous Simulation Environment for Autonomous Obstacle Avoidance Car.
Generates customizable indoor arenas with walls and obstacle pillars.
"""

import math
import random
import numpy as np
from typing import List, Tuple, Dict, Optional

from simulation.vehicle import Vehicle
from simulation.sensors import SensorArray, PRESET_CARDINAL_4
from reinforcement_learning.rewards import RewardCalculator

class ObstacleCourse:
    """Helper to generate segments for arena walls and rectangular obstacles."""
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
        """A realistic indoor obstacle layout with columns, walls, and narrow passages."""
        segments = ObstacleCourse.create_arena_segments(arena_w, arena_h)
        # Pillar 1
        segments.extend(ObstacleCourse.create_box_segments(60, 60, 40, 40))
        # Pillar 2
        segments.extend(ObstacleCourse.create_box_segments(200, 60, 40, 40))
        # Central divider / barrier
        segments.extend(ObstacleCourse.create_box_segments(120, 140, 60, 30))
        # Pillar 3
        segments.extend(ObstacleCourse.create_box_segments(60, 200, 40, 40))
        # Pillar 4
        segments.extend(ObstacleCourse.create_box_segments(200, 200, 40, 40))
        return segments


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
        """Resets the car inside the arena in a collision-free location."""
        self.current_step = 0
        self.is_done = False

        if random_spawn:
            # Pick a safe spot away from obstacles
            for _ in range(100):
                rx = random.uniform(30.0, self.arena_w - 30.0)
                ry = random.uniform(30.0, self.arena_h - 30.0)
                heading = random.uniform(0.0, 360.0)
                self.vehicle.reset(rx, ry, heading)
                # Check distances
                dists = self.sensor_array.get_readings(
                    self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.segments, add_noise=False
                )
                if min(dists) > 25.0:
                    break
        else:
            self.vehicle.reset(self.arena_w / 2.0, 40.0, 90.0)

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
        """Collision occurs if vehicle leaves arena bounds or sensor distance is below chassis bumper (<8cm)."""
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

        # Move vehicle
        self.vehicle.step(action, dt=self.dt)

        # Read ultrasonic sensors
        distances = self.sensor_array.get_readings(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.segments, add_noise=True
        )
        state_id, s_tuple = self.sensor_array.get_discrete_state(distances)

        # Collision & termination check
        collision = self.check_collision(distances)
        terminated = collision
        truncated = (self.current_step >= self.max_steps)

        # Compute reward
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

