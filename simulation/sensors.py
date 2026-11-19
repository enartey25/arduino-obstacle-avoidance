"""
Ultrasonic sensor array (HC-SR04) for 4 cardinal directions:
- S0: UP (Front, 0 deg)
- S1: DOWN (Back/Rear, 180 deg)
- S2: LEFT (Left, -90 deg)
- S3: RIGHT (Right, +90 deg)

Includes acoustic cone multi-ray casting (+-8 deg) matching real HC-SR04 beam spread.
Discretizes distances into 3 states (0=Blocked, 1=Near, 2=Clear) -> 81 States total.
"""

import math
import numpy as np
from typing import List, Tuple, Dict

STATE_BLOCKED = 0  # < 15 cm
STATE_NEAR = 1     # 15 - 35 cm
STATE_CLEAR = 2    # > 35 cm

DIST_BLOCKED_THRESH = 18.0  # cm (gives braking distance for motor inertia)
DIST_NEAR_THRESH = 40.0     # cm (starts gentle curving earlier)
MAX_SENSOR_RANGE = 200.0    # cm

# 4 Cardinal directions layout: [UP, DOWN, LEFT, RIGHT]
PRESET_CARDINAL_4 = [
    {"name": "UP (Front)",    "angle_deg":   0.0, "offset_x": 10.0,  "offset_y":  0.0},
    {"name": "DOWN (Back)",   "angle_deg": 180.0, "offset_x": -10.0, "offset_y":  0.0},
    {"name": "LEFT (Left)",   "angle_deg": -90.0, "offset_x": 0.0,   "offset_y": -7.5},
    {"name": "RIGHT (Right)", "angle_deg":  90.0, "offset_x": 0.0,   "offset_y":  7.5},
]

# Acoustic cone angles for multi-ray detection
CONE_BEAM_OFFSETS_DEG = [-8.0, 0.0, 8.0]


class SensorArray:
    def __init__(self, layout: List[Dict] = None, max_range: float = MAX_SENSOR_RANGE,
                 noise_std: float = 0.5):
        self.layout = layout if layout is not None else PRESET_CARDINAL_4
        self.num_sensors = len(self.layout)
        assert self.num_sensors == 4, "Sensor array must have exactly 4 sensors for 81-state encoding."
        self.max_range = max_range
        self.noise_std = noise_std
        self.last_readings = [max_range] * 4

    @staticmethod
    def discretize_distance(dist_cm: float) -> int:
        """Categorize raw distance into 0 (Blocked), 1 (Near), or 2 (Clear)."""
        if dist_cm < DIST_BLOCKED_THRESH:
            return STATE_BLOCKED
        elif dist_cm <= DIST_NEAR_THRESH:
            return STATE_NEAR
        else:
            return STATE_CLEAR

    @staticmethod
    def encode_state(s0: int, s1: int, s2: int, s3: int) -> int:
        """
        Encodes 4 discrete sensor values into a unique state ID: 0..80.
        state = s0*27 + s1*9 + s2*3 + s3
        where s0=UP, s1=DOWN, s2=LEFT, s3=RIGHT.
        """
        return s0 * 27 + s1 * 9 + s2 * 3 + s3

    @staticmethod
    def decode_state(state_id: int) -> Tuple[int, int, int, int]:
        """Decodes state ID (0..80) into (s_up, s_down, s_left, s_right)."""
        s0 = state_id // 27
        rem0 = state_id % 27
        s1 = rem0 // 9
        rem1 = rem0 % 9
        s2 = rem1 // 3
        s3 = rem1 % 3
        return (s0, s1, s2, s3)

    def cast_ray_to_segment(self, ray_origin: Tuple[float, float], ray_dir: Tuple[float, float],
                            p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Ray-to-line-segment intersection."""
        rx, ry = ray_origin
        rdx, rdy = ray_dir
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1

        det = rdx * dy - rdy * dx
        if abs(det) < 1e-8:
            return self.max_range

        t = ((x1 - rx) * dy - (y1 - ry) * dx) / det
        u = ((x1 - rx) * rdy - (y1 - ry) * rdx) / det

        if t > 0.0 and 0.0 <= u <= 1.0:
            return min(t, self.max_range)
        return self.max_range

    def get_readings(self, vehicle_x: float, vehicle_y: float, vehicle_heading: float,
                     segments: List[Tuple[Tuple[float, float], Tuple[float, float]]],
                     add_noise: bool = True) -> List[float]:
        """
        Simulates ultrasonic acoustic cone ray-casting for UP, DOWN, LEFT, RIGHT sensors.
        Evaluates sub-beams across the cone to simulate ultrasonic reflection.
        """
        readings = []
        cos_h = math.cos(vehicle_heading)
        sin_h = math.sin(vehicle_heading)

        for sensor in self.layout:
            off_x = sensor["offset_x"]
            off_y = sensor["offset_y"]
            origin_x = vehicle_x + (off_x * cos_h - off_y * sin_h)
            origin_y = vehicle_y + (off_x * sin_h + off_y * cos_h)

            sensor_base_angle = vehicle_heading + math.radians(sensor["angle_deg"])

            closest_dist = self.max_range
            # Multi-ray cone coverage
            for cone_offset in CONE_BEAM_OFFSETS_DEG:
                sub_angle = sensor_base_angle + math.radians(cone_offset)
                ray_dir = (math.cos(sub_angle), math.sin(sub_angle))

                for seg in segments:
                    dist = self.cast_ray_to_segment((origin_x, origin_y), ray_dir, seg[0], seg[1])
                    if dist < closest_dist:
                        closest_dist = dist

            if add_noise and self.noise_std > 0.0:
                noise = np.random.normal(0.0, self.noise_std)
                closest_dist = max(2.0, min(self.max_range, closest_dist + noise))

            readings.append(closest_dist)

        self.last_readings = readings
        return readings

    def get_discrete_state(self, distances: List[float]) -> Tuple[int, Tuple[int, int, int, int]]:
        """Converts continuous readings to discrete state (s0, s1, s2, s3) and state_id."""
        s0 = self.discretize_distance(distances[0]) # UP
        s1 = self.discretize_distance(distances[1]) # DOWN
        s2 = self.discretize_distance(distances[2]) # LEFT
        s3 = self.discretize_distance(distances[3]) # RIGHT
        state_id = self.encode_state(s0, s1, s2, s3)
        return state_id, (s0, s1, s2, s3)
