"""
Reward function for 4-directional autonomous navigation:
Sensors: UP, DOWN, LEFT, RIGHT
Actions: UP (Forward), DOWN (Backward), LEFT, RIGHT
"""

from typing import List

class RewardCalculator:
    def __init__(self, collision_penalty: float = -60.0):
        self.collision_penalty = collision_penalty

    def calculate_reward(self, action: int, discrete_states: tuple,
                         continuous_distances: List[float],
                         collision: bool) -> float:
        """
        action: 0=UP (Forward), 1=DOWN (Backward), 2=LEFT, 3=RIGHT
        discrete_states: (s_up, s_down, s_left, s_right) where 0=Blocked (<15cm), 1=Near (15-35cm), 2=Clear (>35cm)
        continuous_distances: [d_up, d_down, d_left, d_right] in cm
        collision: True if vehicle physically hit obstacle/wall (<10cm)
        """
        if collision:
            return self.collision_penalty

        s_up, s_down, s_left, s_right = discrete_states
        reward = 0.0

        # Action 0: UP (Forward)
        if action == 0:
            if s_up == 2:      # Clear ahead
                reward += 3.5
            elif s_up == 1:    # Near obstacle ahead
                reward -= 2.0
            else:              # Blocked obstacle ahead (<15cm)
                reward -= 15.0

        # Action 1: DOWN (Backward)
        elif action == 1:
            if s_down == 0:    # Blocked behind
                reward -= 15.0
            elif s_down == 1:  # Near behind
                reward -= 2.0
            else:              # Clear behind
                if s_up <= 1:  # Front was obstructed -> great backing out maneuver!
                    reward += 3.0
                else:          # Front was clear -> unnecessary reversing
                    reward -= 1.0

        # Action 2: LEFT
        elif action == 2:
            if s_left == 0:
                reward -= 5.0  # Turning into blocked wall
            elif s_left == 2:
                if s_up <= 1:  # Front obstructed, left clear -> good evasion!
                    reward += 3.0
                else:
                    reward -= 0.5

        # Action 3: RIGHT
        elif action == 3:
            if s_right == 0:
                reward -= 5.0  # Turning into blocked wall
            elif s_right == 2:
                if s_up <= 1:  # Front obstructed, right clear -> good evasion!
                    reward += 3.0
                else:
                    reward -= 0.5

        # Distance clearance bonus
        avg_dist = sum(continuous_distances) / len(continuous_distances)
        reward += (avg_dist / 100.0) * 0.5

        return reward
