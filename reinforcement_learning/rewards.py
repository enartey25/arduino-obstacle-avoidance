"""
Reward function for continuous-like navigation with 7 smooth actions:
0: FORWARD_FAST, 1: FORWARD_SLOW, 2: CURVE_LEFT, 3: CURVE_RIGHT,
4: SPIN_LEFT,    5: SPIN_RIGHT,   6: REVERSE
"""

from typing import List

class RewardCalculator:
    def __init__(self, collision_penalty: float = -100.0):
        self.collision_penalty = collision_penalty

    def calculate_reward(self, action: int, discrete_states: tuple,
                         continuous_distances: List[float],
                         collision: bool) -> float:
        if collision:
            return self.collision_penalty

        s_up, s_down, s_left, s_right = discrete_states
        reward = 0.0

        # Scenario 1: Path ahead is completely CLEAR (s_up == 2)
        if s_up == 2:
            if action == 0:     # FORWARD_FAST - Strongly encouraged
                reward += 5.0
            elif action == 1:   # FORWARD_SLOW - Acceptable but sub-optimal in open field
                reward += 2.5
            elif action in (2, 3): # CURVE_LEFT / CURVE_RIGHT - Gentle arc
                reward += 1.0
            elif action in (4, 5): # SPIN_LEFT / SPIN_RIGHT - Discouraged in open space
                reward -= 2.0
            elif action == 6:   # REVERSE - Unnecessary
                reward -= 2.5

        # Scenario 2: Obstacle NEAR in front (s_up == 1)
        elif s_up == 1:
            if action == 0:     # FORWARD_FAST - Dangerous
                reward -= 5.0
            elif action in (2, 3): # Smooth sweeping curve away from obstacle
                if (s_left > s_right and action == 2) or (s_right > s_left and action == 3):
                    reward += 5.0  # Perfect continuous curve into open space
                elif s_left == 2 and action == 2:
                    reward += 4.5
                elif s_right == 2 and action == 3:
                    reward += 4.5
                else:
                    reward += 1.5
            elif action == 1:   # FORWARD_SLOW - Cautious approach
                reward += 1.0
            elif action in (4, 5): # Sharp pivot
                if (s_left > s_right and action == 4) or (s_right > s_left and action == 5):
                    reward += 3.0
                else:
                    reward += 1.0
            elif action == 6:   # REVERSE
                if s_down == 2 and s_left <= 1 and s_right <= 1:
                    reward += 4.0
                else:
                    reward -= 1.0

        # Scenario 3: Obstacle BLOCKED in front (s_up == 0) - Immediate evasive action
        elif s_up == 0:
            if action in (0, 1): # Forward into wall
                reward -= 25.0
            elif action in (2, 3): # Gentle curve may not be sharp enough for <15cm
                reward -= 2.0
            elif action in (4, 5): # Sharp spin pivot away from wall
                if (s_left > s_right and action == 4) or (s_right > s_left and action == 5):
                    reward += 5.5
                elif s_left == 2 and action == 4:
                    reward += 5.0
                elif s_right == 2 and action == 5:
                    reward += 5.0
                else:
                    reward += 2.0
            elif action == 6:   # REVERSE
                if s_down == 2:
                    reward += 5.5  # Ideal dead-end escape
                else:
                    reward -= 5.0

        # Proximity penalization if side or rear is critically close
        if s_down == 0 and action == 6:
            reward -= 15.0
        if s_left == 0 and action in (2, 4):
            reward -= 5.0
        if s_right == 0 and action in (3, 5):
            reward -= 5.0

        # Distance clearance bonus
        avg_dist = sum(continuous_distances) / len(continuous_distances)
        reward += (avg_dist / 100.0) * 0.5

        return reward
