# CXRLGym: ROS 2 Reinforcement Learning Environment

The **CXRLGym** module provides a ROS 2-integrated [Gymnasium](https://gymnasium.farama.org/) environment for reinforcement learning (RL) with multiple robots.

It acts as a bridge between the CLIPS-Executive and standard RL frameworks, allowing agents to perceive and act in a symbolic robot environment using familiar Gym interfaces.

## Overview

`CXRLGym` extends `gymnasium.Env` to provide a ROS-driven reinforcement learning interface.
It automatically constructs its **observation** and **action spaces** by querying ROS 2 services and actions.

### Key Features

- Dynamic observation and action spaces generated from ROS 2 services
- Multi-robot management and coordination
- Compatible with Gym-based RL libraries (e.g., Stable Baselines3)
- ROS 2 actions for reset, robot allocation, and action execution
- Service-based RL mode switching and state creation
