"""
Vehicle kinematics model for a 4-direction differential drive Arduino robot car.
Movements: UP (Forward), DOWN (Backward), LEFT (Spin Left), RIGHT (Spin Right).
Physical specs:
- Width (track): ~15 cm
- Length: ~20 cm
- Control frequency: 10 Hz (dt = 0.1s, matches Arduino 100ms loop)
"""

import math
from typing import Tuple

class Vehicle:
    # 4 Discrete Actions matching Left, Right, Up, Down
    ACTION_UP = 0     # Forward
    ACTION_DOWN = 1   # Backward / Reverse
    ACTION_LEFT = 2   # Turn / Spin Left
    ACTION_RIGHT = 3  # Turn / Spin Right

    ACTION_NAMES = [
        "UP (FORWARD)",
        "DOWN (BACKWARD)",
        "LEFT",
        "RIGHT"
    ]

    # Speeds in cm/s (calibrated to DC gear motors with L298N PWM)
    SPEED_FWD = 22.0    # PWM ~ 200
    SPEED_REV = 18.0    # PWM ~ 180
    SPEED_SPIN = 15.0   # PWM ~ 160

    def __init__(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0,
                 width: float = 15.0, length: float = 20.0):
        self.x = x
        self.y = y
        self.heading = math.radians(heading_deg)  # Radians
        self.width = width
        self.length = length
        self.v_left = 0.0
        self.v_right = 0.0
        self.last_action = self.ACTION_UP

    def reset(self, x: float, y: float, heading_deg: float):
        self.x = x
        self.y = y
        self.heading = math.radians(heading_deg)
        self.v_left = 0.0
        self.v_right = 0.0
        self.last_action = self.ACTION_UP

    def step(self, action: int, dt: float = 0.1) -> Tuple[float, float, float]:
        """
        Applies action using differential drive kinematics:
        - UP: Both wheels forward
        - DOWN: Both wheels reverse
        - LEFT: Left reverse, Right forward (in-place counter-clockwise spin)
        - RIGHT: Left forward, Right reverse (in-place clockwise spin)
        """
        self.last_action = action

        if action == self.ACTION_UP:
            self.v_left = self.SPEED_FWD
            self.v_right = self.SPEED_FWD
        elif action == self.ACTION_DOWN:
            self.v_left = -self.SPEED_REV
            self.v_right = -self.SPEED_REV
        elif action == self.ACTION_LEFT:
            self.v_left = -self.SPEED_SPIN
            self.v_right = self.SPEED_SPIN
        elif action == self.ACTION_RIGHT:
            self.v_left = self.SPEED_SPIN
            self.v_right = -self.SPEED_SPIN
        else:
            self.v_left = 0.0
            self.v_right = 0.0

        # Differential drive forward kinematics
        linear_v = (self.v_right + self.v_left) / 2.0
        angular_w = (self.v_right - self.v_left) / self.width  # rad/s

        self.heading = (self.heading + angular_w * dt) % (2.0 * math.pi)
        dx = linear_v * math.cos(self.heading) * dt
        dy = linear_v * math.sin(self.heading) * dt

        self.x += dx
        self.y += dy

        return dx, dy, angular_w * dt

    def get_bounding_radius(self) -> float:
        """Collision bounding circle radius (in cm)."""
        return math.hypot(self.width / 2.0, self.length / 2.0)
