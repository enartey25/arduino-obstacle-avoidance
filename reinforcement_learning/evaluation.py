"""
Evaluation benchmark for 4-direction trained Q-Learning policy (UP, DOWN, LEFT, RIGHT).
"""

import os
import csv
import numpy as np

from simulation.environment import CarArenaEnv
from simulation.sensors import PRESET_CARDINAL_4
from simulation.renderer import ArenaRenderer
from reinforcement_learning.q_learning import QLearningAgent

def evaluate_policy(agent: QLearningAgent = None,
                    q_table_path: str = "results/q_table.npy",
                    num_test_episodes: int = 100,
                    max_steps: int = 250,
                    visualize: bool = False,
                    results_dir: str = "results") -> dict:
    if agent is None:
        if not os.path.exists(q_table_path):
            raise FileNotFoundError(f"Q-table not found at {q_table_path}. Run training first.")
        agent = QLearningAgent()
        agent.load(q_table_path)

    policy = agent.get_policy()
    env = CarArenaEnv(sensor_layout=PRESET_CARDINAL_4, max_steps_per_episode=max_steps)
    renderer = ArenaRenderer(headless=(not visualize)) if visualize else None

    os.makedirs(results_dir, exist_ok=True)
    csv_path = os.path.join(results_dir, "evaluation_results.csv")
    csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["episode", "total_reward", "steps_survived", "collided", "success"])

    total_rewards = []
    survival_steps = []
    collisions = []
    action_counts = np.zeros(4, dtype=np.int32)

    print("=" * 65)
    print(f"Running Deterministic Evaluation ({num_test_episodes} test episodes)")
    print("=" * 65)

    for ep in range(1, num_test_episodes + 1):
        state, info = env.reset(random_spawn=True)
        ep_reward = 0.0
        collided = False

        for step in range(max_steps):
            action = int(policy[state])
            action_counts[action] += 1

            next_state, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward

            if visualize and renderer:
                action_name = env.vehicle.ACTION_NAMES[action]
                renderer.render(env, action_name=action_name, reward=ep_reward, fps=30)

            state = next_state

            if terminated:
                collided = True
                break
            if truncated:
                break

        success = not collided and (info["step"] >= max_steps)
        total_rewards.append(ep_reward)
        survival_steps.append(info["step"])
        collisions.append(1 if collided else 0)

        writer.writerow([ep, round(ep_reward, 2), info["step"], int(collided), int(success)])

    csv_file.close()
    if renderer:
        renderer.close()

    success_rate = (1.0 - np.mean(collisions)) * 100.0
    mean_reward = np.mean(total_rewards)
    mean_steps = np.mean(survival_steps)

    print(f"Evaluation Complete:")
    print(f"  Success Rate (No Collision): {success_rate:.1f}%")
    print(f"  Mean Steps Survived:         {mean_steps:.1f} / {max_steps}")
    print(f"  Mean Episode Reward:         {mean_reward:.1f}")
    print(f"  Action Distribution:")
    action_labels = ["UP (FORWARD)", "DOWN (BACKWARD)", "LEFT", "RIGHT"]
    total_actions = sum(action_counts)
    for act_idx, act_name in enumerate(action_labels):
        pct = (action_counts[act_idx] / total_actions) * 100.0 if total_actions > 0 else 0
        print(f"    - {act_name:16s}: {action_counts[act_idx]:5d} ({pct:5.1f}%)")
    print("-" * 65)
    print(f"Evaluation log saved to: {csv_path}")

    return {
        "success_rate": success_rate,
        "mean_steps": mean_steps,
        "mean_reward": mean_reward,
        "action_counts": action_counts.tolist()
    }

if __name__ == "__main__":
    evaluate_policy()
