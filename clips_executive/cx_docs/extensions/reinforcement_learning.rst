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

An example Gymnasium environment configuration can be found `here <rl/environment.html>`_.


Overview
++++++++

This extension leverages the `Gymnasium`_ framework, a modern and extensible toolkit for RL environments.
It provides a Python-based ROS node that bridges the Gymnasium API with CLIPS, allowing bidirectional communication between the rule-based and learning components.

CLIPS can act as the high-level controller, managing symbolic knowledge, rules, and goals, while the RL node handles low-level control or adaptive behaviors learned from experience.

Additionally, this extension includes utilities to:

 - wrap custom ROS environments as Gymnasium environments,
 - synchronize state and reward information with the CLIPS fact base,
 - and train or evaluate policies within the |CX| ecosystem.

Content
+++++++

.. toctree::
   :maxdepth: 2

   rl/usage.rst
   rl/rl_clips.rst
