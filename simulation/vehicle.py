"""
Vehicle kinematics model with continuous-like smooth motion and variable motor speeds.
Features:
- 7 Actions: Fast Cruise, Cautious Forward, Sweeping Left Curve, Sweeping Right Curve,
             Tight Pivot Left, Tight Pivot Right, Smooth Reverse.
- Internal motor speed slew-rate filter (momentum smoothing) to eliminate jerking.
"""

import math
from typing import Tuple

class Vehicle:
    # 7 Smooth / Variable Actions
    ACTION_FORWARD_FAST = 0   # High-speed cruise in open space
    ACTION_FORWARD_SLOW = 1   # Cautious glide near obstacles
    ACTION_CURVE_LEFT   = 2   # Smooth sweeping left arc (both wheels moving forward)
    ACTION_CURVE_RIGHT  = 3   # Smooth sweeping right arc (both wheels moving forward)
    ACTION_SPIN_LEFT    = 4   # Tight in-place pivot left (for tight corner escape)
    ACTION_SPIN_RIGHT   = 5   # Tight in-place pivot right (for tight corner escape)
    ACTION_REVERSE      = 6   # Smooth reverse

    ACTION_NAMES = [
        "FORWARD_FAST",
        "FORWARD_SLOW",
        "CURVE_LEFT",
        "CURVE_RIGHT",
        "SPIN_LEFT",
        "SPIN_RIGHT",
        "REVERSE"
    ]

    # Target wheel speeds in cm/s
    SPEED_CRUISE = 20.0   # Fast forward (PWM ~215)
    SPEED_GLIDE  = 12.0   # Cautious forward (PWM ~135)
    SPEED_ARC_LO =  9.0   # Inner wheel on smooth curves (PWM ~110)
    SPEED_ARC_HI = 19.0   # Outer wheel on smooth curves (PWM ~200)
    SPEED_PIVOT  = 13.0   # Pivot speed (PWM ~145)
    SPEED_REV    = 13.0   # Reverse speed (PWM ~145)

    # Momentum filter factor (0.0 to 1.0; 0.5 = smooth acceleration/deceleration)
    ACCEL_FILTER = 0.50

    def __init__(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0,
                 width: float = 15.0, length: float = 20.0):
        self.x = x
        self.y = y
        self.heading = math.radians(heading_deg)
        self.width = width
        self.length = length
        self.v_left = 0.0
        self.v_right = 0.0
        self.target_v_left = 0.0
        self.target_v_right = 0.0
        self.last_action = self.ACTION_FORWARD_FAST

    def reset(self, x: float, y: float, heading_deg: float):
        self.x = x
        self.y = y
        self.heading = math.radians(heading_deg)
        self.v_left = 0.0
        self.v_right = 0.0
        self.target_v_left = 0.0
        self.target_v_right = 0.0
        self.last_action = self.ACTION_FORWARD_FAST

    def step(self, action: int, dt: float = 0.1) -> Tuple[float, float, float]:
        """
        Calculates target wheel speeds based on action, applies acceleration smoothing,
        and updates vehicle pose using differential drive kinematics.
        """
        self.last_action = action

        if action == self.ACTION_FORWARD_FAST:
            self.target_v_left = self.SPEED_CRUISE
            self.target_v_right = self.SPEED_CRUISE
        elif action == self.ACTION_FORWARD_SLOW:
            self.target_v_left = self.SPEED_GLIDE
            self.target_v_right = self.SPEED_GLIDE
        elif action == self.ACTION_CURVE_LEFT:
            # Sweeping curve: inner wheel slow, outer wheel fast
            self.target_v_left = self.SPEED_ARC_LO
            self.target_v_right = self.SPEED_ARC_HI
        elif action == self.ACTION_CURVE_RIGHT:
            # Sweeping curve: outer wheel fast, inner wheel slow
            self.target_v_left = self.SPEED_ARC_HI
            self.target_v_right = self.SPEED_ARC_LO
        elif action == self.ACTION_SPIN_LEFT:
            self.target_v_left = -self.SPEED_PIVOT
            self.target_v_right = self.SPEED_PIVOT
        elif action == self.ACTION_SPIN_RIGHT:
            self.target_v_left = self.SPEED_PIVOT
            self.target_v_right = -self.SPEED_PIVOT
        elif action == self.ACTION_REVERSE:
            self.target_v_left = -self.SPEED_REV
            self.target_v_right = -self.SPEED_REV
        else:
            self.target_v_left = 0.0
            self.target_v_right = 0.0

        # Continuous-like momentum smoothing
        self.v_left += (self.target_v_left - self.v_left) * self.ACCEL_FILTER
        self.v_right += (self.target_v_right - self.v_right) * self.ACCEL_FILTER

        linear_v = (self.v_right + self.v_left) / 2.0
        angular_w = (self.v_right - self.v_left) / self.width

        self.heading = (self.heading + angular_w * dt) % (2.0 * math.pi)
        dx = linear_v * math.cos(self.heading) * dt
        dy = linear_v * math.sin(self.heading) * dt

        self.x += dx
        self.y += dy

        return dx, dy, angular_w * dt

    def get_bounding_radius(self) -> float:
        return math.hypot(self.width / 2.0, self.length / 2.0)
