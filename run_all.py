"""
Autonomous RL Car: End-to-End Sprint Runner.
Executes complete pipeline: Training -> Evaluation -> Arduino Header Export -> Trajectory Generation.
"""

import os
import sys
import time

from reinforcement_learning.training import train_agent
from reinforcement_learning.evaluation import evaluate_policy
from deployment.export_policy import export_policy_header
from demo import run_interactive_demo

def main():
    print("=" * 70)
    print(" AUTONOMOUS RL CAR: SIMULATION-TO-ARDUINO SPRINT PIPELINE")
    print(" Target: Arduino Uno (ATmega328P) + L298N + 4x HC-SR04")
    print("=" * 70)

    # 1. Training
    print("\n>>> STAGE 1: Q-Learning Policy Training (2,500 episodes)...")
    agent = train_agent(num_episodes=2500, max_steps_per_episode=200)

    # 2. Evaluation
    print("\n>>> STAGE 2: Deterministic Policy Evaluation (100 episodes)...")
    eval_metrics = evaluate_policy(agent=agent, num_test_episodes=100)

    # 3. Policy Export
    print("\n>>> STAGE 3: Exporting Policy to C++ Flash Header (policy.h)...")
    header_path = export_policy_header()

    # 4. Trajectory Visualization
    print("\n>>> STAGE 4: Generating Sample Navigation Trajectory...")
    run_interactive_demo()

    print("\n" + "=" * 70)
    print(" ALL DEPLOYMENT ARTIFACTS GENERATED SUCCESSFULLY!")
    print("=" * 70)
    print(f" 1. Training Log:       results/training_rewards.csv")
    print(f" 2. Learning Curves:    results/training_curve.png")
    print(f" 3. Evaluation Log:     results/evaluation_results.csv")
    print(f" 4. Sample Trajectory:  results/demo_trajectory.png")
    print(f" 5. C++ Policy Header:  deployment/policy.h and arduino/policy.h")
    print(f" 6. Arduino Firmware:   arduino/arduino.ino, motors.cpp, ultrasonic.cpp")
    print("=" * 70)

if __name__ == "__main__":
    main()

