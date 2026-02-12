Usage
#####

This document assumes you have the |CX| installed and sourced in your environment.

Aside from the regular |CX| node, the RL extension additionally requires a node for the reinforcement learning agent.
We provide the **CXRLNode** node for this purpose which  deploys a variant of multi-robot proximal policy optimization with invalid action masking via the :docsite:`cx_rl_multi_robot_mppo` package.

Starting the CX RL Node
************************

Aside from the regular |CX| node, the RL extension additionally requires a RL node that implemnts  **PDDL Manager** node (a lifecycle node with a bond timer).

To just launch a standalone node, just run the following command:

.. code-block:: bash

  ros2 run cx_pddl_manager pddl_manager

A more practical way to to start the node is to utilize the launch file of the `cx_pddl_bringup`, which wraps the |CX| launch file and additionally starts the PDDL Manager alongside of it:

.. code-block:: bash

  ros2 launch cx_pddl_bringup cx_pddl_launch.py

.. hint::

  Use the `--show-args` option to learn about all parameters that the launch file provides.


Example
*******

An example application is provided through the :docsite:`cx_rl_bringup` package and can be started via:
The following example showcases how an RL agent can be trained and utilzied with CLIPS-based execution. Here, a simple blocksworld domain is considered, where four blocks A, B, C and D need to be stacked using the actions pickup (picking up a block) and stack (placing one block on top of another block).

.. code-block:: bash

  ros2 launch cx_rl_bringup agent.launch.py

.. hint::

  Use the `--show-args` option to learn about all parameters that the launch file provides.


Check out the :ref:`Intefacing Guide <reinforcement_learning/rl_clips.html>` page to learn how to write your own CLIPS applications while leveraging reinforcement learning.
