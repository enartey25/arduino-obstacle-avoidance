"""
Training loop for Tabular Q-Learning Autonomous Obstacle Avoidance Car.
Runs fast vectorized/headless training, logs metrics, and exports artifacts.
"""

import os
import csv
import time
import numpy as np
import matplotlib.pyplot as plt

from simulation.environment import CarArenaEnv
from simulation.sensors import PRESET_CARDINAL_4
from reinforcement_learning.q_learning import QLearningAgent

def train_agent(num_episodes: int = 3000,
                max_steps_per_episode: int = 200,
                results_dir: str = "results",
                sensor_layout = None) -> QLearningAgent:
    os.makedirs(results_dir, exist_ok=True)

    layout = sensor_layout if sensor_layout is not None else PRESET_CARDINAL_4
    env = CarArenaEnv(sensor_layout=layout, max_steps_per_episode=max_steps_per_episode)
    agent = QLearningAgent(alpha=0.20, gamma=0.93, epsilon_start=1.0, epsilon_decay=0.998, epsilon_min=0.02)

    csv_path = os.path.join(results_dir, "training_rewards.csv")
    csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["episode", "total_reward", "steps", "collision", "epsilon"])

    rewards_history = []
    steps_history = []
    collisions_history = []

    print("=" * 65)
    print(f"Starting Q-Learning Training ({num_episodes} episodes)")
    print(f"State space: {agent.NUM_STATES} states | Action space: {agent.NUM_ACTIONS} actions")
    print("=" * 65)

    start_time = time.time()

    for episode in range(1, num_episodes + 1):
        state, info = env.reset(random_spawn=True)
        total_reward = 0.0
        collided = False

        for step in range(max_steps_per_episode):
            action = agent.choose_action(state, greedy=False)
            next_state, reward, terminated, truncated, info = env.step(action)

            agent.update(state, action, reward, next_state, terminated)
            state = next_state
            total_reward += reward

            if terminated:
                collided = True
                break
            if truncated:
                break

        agent.decay_epsilon()

        rewards_history.append(total_reward)
        steps_history.append(info["step"])
        collisions_history.append(1 if collided else 0)

        writer.writerow([episode, round(total_reward, 2), info["step"], int(collided), round(agent.epsilon, 4)])

        if episode % 300 == 0 or episode == num_episodes:
            csv_file.flush()
            recent_reward = np.mean(rewards_history[-100:])
            recent_steps = np.mean(steps_history[-100:])
            collision_pct = np.mean(collisions_history[-100:]) * 100.0
            print(f"Episode {episode:4d}/{num_episodes} | Avg Reward: {recent_reward:7.1f} | "
                  f"Avg Steps: {recent_steps:5.1f} | Collisions: {collision_pct:4.1f}% | Epsilon: {agent.epsilon:.3f}", flush=True)

    elapsed = time.time() - start_time
    csv_file.close()

    print("-" * 65)
    print(f"Training completed in {elapsed:.2f} seconds ({num_episodes / elapsed:.1f} episodes/sec).")

    q_table_path = os.path.join(results_dir, "q_table.npy")
    agent.save(q_table_path)
    print(f"Q-table saved to: {q_table_path}")

    plot_training_curves(rewards_history, steps_history, collisions_history, results_dir)

    return agent


def plot_training_curves(rewards, steps, collisions, results_dir):
    """Generates informative training performance curves."""
    window = 50
    def moving_avg(arr, w):
        return np.convolve(arr, np.ones(w)/w, mode='valid')

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    episodes = range(1, len(rewards) + 1)

    # 1. Rewards
    ax1.plot(episodes, rewards, color="lightsteelblue", alpha=0.4, label="Raw Episode Reward")
    if len(rewards) >= window:
        ax1.plot(range(window, len(rewards) + 1), moving_avg(rewards, window),
                 color="navy", linewidth=2, label=f"{window}-Episode Moving Avg")
    ax1.set_ylabel("Reward")
    ax1.set_title("Autonomous Car Q-Learning Training Performance", fontsize=14, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper left")

    # 2. Survival Steps
    ax2.plot(episodes, steps, color="lightsalmon", alpha=0.4, label="Steps Survived")
    if len(steps) >= window:
        ax2.plot(range(window, len(steps) + 1), moving_avg(steps, window),
                 color="darkred", linewidth=2, label=f"{window}-Episode Moving Avg")
    ax2.set_ylabel("Steps Survived")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left")

    # 3. Collision Rate
    if len(collisions) >= window:
        coll_pct = moving_avg(collisions, window) * 100.0
        ax3.plot(range(window, len(collisions) + 1), coll_pct,
                 color="crimson", linewidth=2, label="Collision Rate %")
    ax3.set_ylabel("Collision Rate (%)")
    ax3.set_xlabel("Episode")
    ax3.set_ylim(-5, 105)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    plt.tight_layout()
    plot_path = os.path.join(results_dir, "training_curve.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"Learning curves plot saved to: {plot_path}")


if __name__ == "__main__":
    train_agent(num_episodes=3000)
