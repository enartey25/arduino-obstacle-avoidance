"""
Reward function for 4-directional autonomous navigation:
Sensors: UP, DOWN, LEFT, RIGHT
Actions: UP (Forward), DOWN (Backward), LEFT, RIGHT
Optimized to eliminate spinning degeneration and maximize collision-free survival.
"""

from typing import List

class RewardCalculator:
    def __init__(self, collision_penalty: float = -100.0):
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
        d_up, d_down, d_left, d_right = continuous_distances
        reward = 0.0

        # Scenario 1: Path ahead is completely CLEAR (s_up == 2)
        if s_up == 2:
            if action == 0:     # UP (Forward) - Strongly desired behavior
                reward += 4.5
            elif action == 1:   # DOWN (Reverse) - Unnecessary in open field
                reward -= 2.0
            else:               # Spin left / right - Discouraged when road is wide open
                reward -= 1.5

        # Scenario 2: Obstacle NEAR in front (s_up == 1)
        elif s_up == 1:
            if action == 0:     # Driving into near obstacle
                reward -= 4.0
            elif action in (2, 3):  # Turn away
                if (s_left > s_right and action == 2) or (s_right > s_left and action == 3):
                    reward += 4.0   # Steered into more open side
                elif s_left == 2 and action == 2:
                    reward += 3.5
                elif s_right == 2 and action == 3:
                    reward += 3.5
                else:
                    reward += 1.0
            elif action == 1:   # Reverse
                if s_down == 2 and s_left <= 1 and s_right <= 1:
                    reward += 3.5   # Good back-up decision if sides are tight
                else:
                    reward -= 0.5

        # Scenario 3: Obstacle BLOCKED in front (s_up == 0) - Critical evasion needed
        elif s_up == 0:
            if action == 0:     # Severe penalty for driving into blocked wall
                reward -= 25.0
            elif action in (2, 3):  # Spin turn
                if (s_left > s_right and action == 2) or (s_right > s_left and action == 3):
                    reward += 5.0
                elif s_left == 2 and action == 2:
                    reward += 4.5
                elif s_right == 2 and action == 3:
                    reward += 4.5
                else:
                    reward += 1.5
            elif action == 1:   # Reverse out
                if s_down == 2:
                    reward += 5.0   # Optimal escape maneuver!
                else:
                    reward -= 5.0   # Backing into blocked rear

        # Proximity penalization if side or rear is critically close
        if s_down == 0 and action == 1:
            reward -= 15.0
        if s_left == 0 and action == 2:
            reward -= 5.0
        if s_right == 0 and action == 3:
            reward -= 5.0

        # Clearance bonus
        avg_dist = sum(continuous_distances) / len(continuous_distances)
        reward += (avg_dist / 100.0) * 0.5

        return reward
