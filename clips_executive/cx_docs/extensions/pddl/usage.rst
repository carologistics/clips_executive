Usage
#####
This document assumes you have the |CX| installed and sourced in your environment.

Starting the PDDL Manager
*************************

Aside from the regular |CX| node, the PDDL extension additionally requires the **PDDL Manager** node (a lifecycle node with a bond timer).

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

The following example showcases simple interactions between CLIPS and PDDL by running an example agent:

.. code-block:: bash

   ros2 launch cx_pddl_bringup cx_pddl_launch.py manager_config:=pddl_agents/structured_agent.yaml


Check out the :ref:`Intefacing Guide <pddl/pddl_clips.html>` page to learn how to write your own CLIPS applications while leveraging PDDL.
