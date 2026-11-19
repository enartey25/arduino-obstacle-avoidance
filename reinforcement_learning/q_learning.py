"""
Tabular Q-Learning Agent for 81 States and 7 Continuous-Like Smooth Actions.
Q-table: 81 x 7.
Policy array: 81 bytes stored in Arduino flash memory (PROGMEM).
"""

import numpy as np

class QLearningAgent:
    NUM_STATES = 81
    NUM_ACTIONS = 7  # 7 Smooth Actions: Fast, Slow, Curve-L, Curve-R, Spin-L, Spin-R, Rev

    def __init__(self,
                 alpha: float = 0.20,
                 gamma: float = 0.93,
                 epsilon_start: float = 1.0,
                 epsilon_decay: float = 0.998,
                 epsilon_min: float = 0.02,
                 optimistic_init: float = 0.0):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min

        # Q-table: (81 states, 7 actions)
        self.q_table = np.full((self.NUM_STATES, self.NUM_ACTIONS), optimistic_init, dtype=np.float32)
        self.state_visits = np.zeros(self.NUM_STATES, dtype=np.int32)

    def choose_action(self, state: int, greedy: bool = False) -> int:
        self.state_visits[state] += 1
        if not greedy and np.random.rand() < self.epsilon:
            return np.random.randint(self.NUM_ACTIONS)
        q_vals = self.q_table[state]
        max_val = np.max(q_vals)
        best_actions = np.where(np.isclose(q_vals, max_val, atol=1e-5))[0]
        return int(np.random.choice(best_actions))

    def update(self, state: int, action: int, reward: float, next_state: int, done: bool):
        current_q = self.q_table[state, action]
        if done:
            target = reward
        else:
            best_future_q = np.max(self.q_table[next_state])
            target = reward + self.gamma * best_future_q

        self.q_table[state, action] += self.alpha * (target - current_q)

    def decay_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def get_best_action(self, state: int) -> int:
        return int(np.argmax(self.q_table[state]))

    def get_policy(self) -> np.ndarray:
        policy = np.zeros(self.NUM_STATES, dtype=np.uint8)
        for s in range(self.NUM_STATES):
            if np.all(np.isclose(self.q_table[s], 0.0)):
                s_up = s // 27
                s_down = (s % 27) // 9
                s_left = (s % 9) // 3
                s_right = s % 3

                if s_up == 2:
                    policy[s] = 0  # FORWARD_FAST
                elif s_up == 1:
                    if s_left > s_right and s_left == 2:
                        policy[s] = 2  # CURVE_LEFT
                    elif s_right > s_left and s_right == 2:
                        policy[s] = 3  # CURVE_RIGHT
                    else:
                        policy[s] = 1  # FORWARD_SLOW
                else:  # s_up == 0 (Blocked)
                    if s_left > s_right:
                        policy[s] = 4  # SPIN_LEFT
                    elif s_right > s_left:
                        policy[s] = 5  # SPIN_RIGHT
                    elif s_down == 2:
                        policy[s] = 6  # REVERSE
                    else:
                        policy[s] = 5  # SPIN_RIGHT
            else:
                policy[s] = np.argmax(self.q_table[s])

        return policy

    def save(self, filepath: str):
        np.save(filepath, self.q_table)

    def load(self, filepath: str):
        self.q_table = np.load(filepath)
