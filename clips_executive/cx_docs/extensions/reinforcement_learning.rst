Reinforcement Learning Integration for CLIPS
============================================

This extension provides an implementation for using Reinforcement Learning (RL) alongside CLIPS.

If you need one of the following features, you may find this useful:

- Adaptive decision-making through trial and error in dynamic environments
- Integration of RL-based policies with CLIPS rules and reasoning
- A modular interface between symbolic reasoning (CLIPS) and sub-symbolic learning (RL)
- Seamless experimentation with Gymnasium-compatible environments in ROS

Background: Reinforcement Learning
++++++++++++++++++++++++++++++++++

Reinforcement Learning (RL) is a paradigm of machine learning in which an agent learns to make decisions by interacting with its environment.
The agent observes the current state, takes actions, and receives rewards that guide its behavior over time.

An RL setup typically consists of:

 - an *environment*, which defines the state space, action space, and transition dynamics,
 - an *agent*, which selects actions based on its current policy,
 - and a *reward function*, which provides feedback on the desirability of each action outcome.

Through repeated interaction, the agent refines its policy to maximize cumulative reward.


Overview
++++++++

The Reinforcement Learning (RL) integration is provided through the CXRLGym class.
It provides a Gym environemnt that utilizes ROS interaces to customize training and execution, which can be called through the |CX| plugins that handle ROS communication.

Additionally, an RL algorithm is needed as well as a node to utilize both.
We provide a multi-robot Maskable Proximal Policy Optimization (MPPO) algorithm thorugh the ``cx_rl_multi_robot_mppo`` package, which alsoprovides a suitable ``cx_rl_node`` to use CXRLGym based environments and the multi-robot MPPO policy to train and apply policies.

In order to reduce the manual overhead, the ``cx_rl_clips`` package provides a CLIPS-based interface for interacting with the PDDL Manager node in ROS 2.
This allows to interact with the CXRLGym manager by simply asserting and monitoring CLIPS facts, without the need to do direct ROS communication (e.g., populating ROS messages or waiting for service feedback).

This makes it possible to:

* Train reinforcement learning agents using ROS
* Integrate symbolic reasoning and learning-based control by combining it with CLIPS
* Evaluate trained models directly on real or simulated robots


Content
+++++++

.. toctree::
   :maxdepth: 2

   rl/usage.rst
   rl/cx_rl.rst
   rl/rl_clips.rst
