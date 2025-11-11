Usage
#####
This document assumes you have the |CX| installed and sourced in your environment.

Utilizing CXRLGym
*****************

Aside from the regular |CX| node, the PDDL extension additionally requires an instance of the **CXRLGym** node.
However, since CXRLGym is just a skeleton, you first need to decide on and implemnent an RL algorithm.
We provide a reference implementation that deploys a variant of multi-robot proximal policy optimization with invalid action masking in the :docsite:`cx_rl_multi_robot_mppo` package.



Example
*******

An example application is provided through the :docsite:`cx_rl_bringup` package and can be started via:
The following example showcases simple interactions between CLIPS and CXRLGym by running an example agent:

.. code-block:: bash

  ros2 launch cx_rl_bringup agent.launch.py

.. hint::

  Use the `--show-args` option to learn about all parameters that the launch file provides.


Check out the :ref:`Intefacing Guide <reinforcement_learning/rl_clips.html>` page to learn how to write your own CLIPS applications while leveraging reinforcement learning.
