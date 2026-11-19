"""
Autonomous RL Car - Interactive Demonstration.
Visualizes the trained Q-learning policy navigating the arena with live sensor rays.
Uses Pygame if installed, or Matplotlib animated playback as fallback.
"""

import sys
import time
import numpy as np

from simulation.environment import CarArenaEnv
from simulation.sensors import PRESET_CARDINAL_4
from simulation.renderer import ArenaRenderer, PYGAME_AVAILABLE
from reinforcement_learning.q_learning import QLearningAgent

def run_interactive_demo(q_table_path="results/q_table.npy", max_steps=500):
    agent = QLearningAgent()
    agent.load(q_table_path)
    policy = agent.get_policy()

    env = CarArenaEnv(sensor_layout=PRESET_CARDINAL_4, max_steps_per_episode=max_steps)

    if PYGAME_AVAILABLE:
        print("Launching Pygame Real-Time Visualizer...")
        renderer = ArenaRenderer(headless=False)
        state, info = env.reset(random_spawn=True)
        total_reward = 0.0

        for step in range(max_steps):
            action = int(policy[state])
            action_name = env.vehicle.ACTION_NAMES[action]
            next_state, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            renderer.render(env, action_name=action_name, reward=total_reward, fps=25)
            state = next_state

            if terminated:
                print(f"Collision at step {step}! Resetting position...")
                time.sleep(0.5)
                state, info = env.reset(random_spawn=True)

        renderer.close()
    else:
        print("Pygame not available. Running headless demo and generating trajectory plot...")
        import matplotlib.pyplot as plt

        state, info = env.reset(random_spawn=False)
        traj_x = [env.vehicle.x]
        traj_y = [env.vehicle.y]

        for step in range(300):
            action = int(policy[state])
            next_state, reward, terminated, truncated, info = env.step(action)
            traj_x.append(env.vehicle.x)
            traj_y.append(env.vehicle.y)
            state = next_state
            if terminated:
                break

        plt.figure(figsize=(8, 8))
        for seg in env.segments:
            p1, p2 = seg
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', linewidth=2)
        plt.plot(traj_x, traj_y, 'b.-', label="Trained RL Path", alpha=0.7)
        plt.scatter([traj_x[0]], [traj_y[0]], c='g', s=100, label="Start", zorder=5)
        plt.scatter([traj_x[-1]], [traj_y[-1]], c='r', s=100, label="End", zorder=5)
        plt.title("Autonomous RL Car - Evaluated Navigation Trajectory")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.legend()
        plt.savefig("results/demo_trajectory.png", dpi=150)
        plt.close()
        print("Saved trajectory map to results/demo_trajectory.png")

if __name__ == "__main__":
    run_interactive_demo()

