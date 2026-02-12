.. _structured_rl_agent:

Tutorial: Building an RL Agent for Blocksworld
##############################################

**Goal:** Use CLIPS to define a reinforcement learning agent for
`blocksworld`_,
generate symbolic actions, assign rewards, and train or execute an RL policy.

**Tutorial level:** Advanced

**Time:** 45–60 minutes

.. contents:: Contents
   :depth: 2
   :local:


Overview
--------

This tutorial demonstrates how to integrate the blocksworld domain with the ``CXRLGym`` reinforcement learning environment using CLIPS.

You will learn how to:

1. figure the CLIPS RL agent and required plugins,
2. Define symbolic observables and actions,
3. Implement the reset lifecycle,
4. Generate candidate RL actions,
5. Execute selected actions and assign rewards,
6. Detect episode termination,
7. Run the agent in both training and execution mode.


Prerequisites
-------------

This tutorial assumes that |CX| is installed and that you are familiar with creating and configuring a custom package using it.

It provides the steps to replicate the default agent of the ``cx_rl_bringup`` package.

Package Structure
-----------------

The following structure should be present, create missing directories and files as depicted

.. code-block:: bash

   mkdir clips params
   touch clips/rl-blocksworld.clp
   touch params/rl_agent.yaml
   touch params/rl_node_config.yaml
   # Resulting package:
   .
   ├── clips
   │   └── rl-blocksword.clp
   ├── CMakeLists.txt
   ├── package.xml
   └── params
       ├── rl_agent.yaml
       └── rl_node_config.yaml


Configuration
-------------

The CLIPS agent is configured via ``rl_agent.yaml``.

.. code-block:: yaml

   clips_manager:
     ros__parameters:
       environments: ["cx_rl_bringup"]
       cx_rl_bringup:
         plugins: ["executive",
                   "ament_index",
                   "ros_msgs",
                   "ros_param",
                   "action_selection",
                   "get_free_robot",
                   "reset_env",
                   "rl_files",
                   "files"]
         log_clips_to_file: true
         watch: ["facts", "rules"]
         redirect_stdout_to_debug: true

       ament_index:
         plugin: "cx::AmentIndexPlugin"

       executive:
         plugin: "cx::ExecutivePlugin"
         publish_on_refresh: false
         assert_time: true
         refresh_rate: 10

       ros_msgs:
         plugin: "cx::RosMsgsPlugin"

       ros_param:
         plugin: "cx::RosParamPlugin"

       reset_env:
         plugin: "cx::CXCxRlInterfacesResetEnvPlugin"

       action_selection:
         plugin: "cx::CXCxRlInterfacesActionSelectionPlugin"

       get_free_robot:
         plugin: "cx::CXCxRlInterfacesGetFreeRobotPlugin"

       rl_files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_rl_clips"]
         batch: [
           "clips/cx_rl_clips/cx-rl.clp",
         ]

       files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_rl_bringup"]
         load: [
           "clips/cx_rl_bringup/rl-blocksworld.clp",
         ]

The following plugins are used:

* ``executive`` – Controls reasoning cycles
* ``ros_msgs`` – Provides ROS communication
* ``reset_env`` – Enables environment reset
* ``action_selection`` – Executes symbolic actions
* ``get_free_robot`` – Handles robot availability
* ``rl_files`` – Loads reusable RL CLIPS interfaces
* ``files`` – Loads the BlocksWorld rule base

.. admonition:: Separate FielLoadPlugins

  The cx_rl.clp file provided by cx_rl_clips must be batch loaded rather than using a standard load. This is because it contains not only deftemplates and rules, but also CLIPS commands that must be executed during loading.

  To accommodate this, we use a separate FileLoadPlugin for batch loading the interface, while user-defined rules (e.g., rl-blocksworld.clp) are loaded using a separate FileLoadPlugin. Loading both the interface and user code with a single plugin would not work, as batch commands are only executed after the plugin processes all load entries.


Defining Environment Logic
--------------------------

The goal is to define an environment consisting of four blocks (``block1``, ``block2``, ``block3`` and ``block4``) that have to be stacked to an ordered tower (``block1`` at the bottom, ``block4`` on top). Initially all blocks lay on the table.
Available actions are to pick up a block from a table and to stack the picked up block onto another block.

This tutorial follows along the steps defined in the :ref:`RL CLIPS interface doucmentation <clips_workflows>`.

Blocksworld Step 0: Configuration via Global Variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Keeping the default node name untouched, the only thing to do is to define some rewards for episod failure or success.

.. code-block:: lisp

  (defglobal
    ?*CX-RL-REWARD-EPISODE-SUCCESS* = 100
    ?*CX-RL-REWARD-EPISODE-FAILURE* = -100
  )


Blocksworld Step 1: Defining the Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A single rule can take care of this step:

.. code-block:: lisp

  (defrule rl-blocksworld-initial-state
    (not (cx-rl-node))
  =>
    (assert
      (rl-observable-type (type robot) (objects robot1))
      (rl-observable-type (type block) (objects block1 block2 block3 block4))
      (rl-observable-predicate (name on-table) (param-names a) (param-types block))
      (rl-observable-predicate (name clear) (param-names a) (param-types block))
      (rl-observable-predicate (name can-hold) (param-names r) (param-types robot))
      (rl-observable-predicate (name holding) (param-names r a) (param-types robot block))
      (rl-observable-predicate (name on) (param-names a b) (param-types block block))
      (rl-observable-predicate (name target-on) (param-names a b) (param-types block block))
      (rl-observable-predicate (name target-on-table) (param-names a) (param-types block))
      (rl-observable-action (name stack) (param-names r b1 b2) (param-types robot block block))
      (rl-observable-action (name pickup) (param-names r b) (param-types robot block))
      (rl-observation (name clear) (param-values block1))
      (rl-observation (name clear) (param-values block2))
      (rl-observation (name clear) (param-values block3))
      (rl-observation (name clear) (param-values block4))
      (rl-observation (name on-table) (param-values block1))
      (rl-observation (name on-table) (param-values block2))
      (rl-observation (name on-table) (param-values block3))
      (rl-observation (name on-table) (param-values block4))
      (rl-observation (name target-on) (param-values block4 block3))
      (rl-observation (name target-on) (param-values block3 block2))
      (rl-observation (name target-on) (param-values block2 block1))
      (rl-observation (name target-on-table) (param-values block1))
      (rl-observation (name can-hold) (param-values robot1))
      (rl-observation (name clear) (param-values block1))
      (rl-observation (name clear) (param-values block2))
      (rl-observation (name clear) (param-values block3))
      (rl-observation (name clear) (param-values block4))
      (rl-robot (name robot1) (waiting TRUE))
      (cx-rl-node (name ?*CX-RL-NODE-NAME*) (mode UNSET))
    )
  )

The rule condition ensures that this rule only fires exactly onces, as it asserts an cx-rl-node fact.
This ensures that it is not accidentally called again after the environment resets during training.

.. code-block:: lisp

  (defrule rl-blocksworld-initial-state
    (not (cx-rl-node))
  =>

In order to define the blocksworld environment, a few parameterized predicates are defined.

.. code-block:: lisp

      (rl-observable-type (type robot) (objects robot1))
      (rl-observable-type (type block) (objects block1 block2 block3 block4))
      (rl-observable-predicate (name on-table) (param-names a) (param-types block))
      (rl-observable-predicate (name clear) (param-names a) (param-types block))
      (rl-observable-predicate (name can-hold) (param-names r) (param-types robot))
      (rl-observable-predicate (name holding) (param-names r a) (param-types robot block))
      (rl-observable-predicate (name on) (param-names a b) (param-types block block))
      (rl-observable-predicate (name target-on) (param-names a b) (param-types block block))
      (rl-observable-predicate (name target-on-table) (param-names a) (param-types block))

Intuitively, this provides the following symbolic representation of the environment:

 - `on-table(blockX)`: The block ``blockX`` is on the table
 - `clear(blockX)`: The block ``blockX`` can be used as stacking base, as no other block is on top if.
 - `can-hold(robotX)`: The robot ``robotX`` is ready to pick up a block.
 - `holding(robotX, blockX)` The robot ``robotX`` is ready is holding ``blockX`` and therfore ready to stack it on another block.
 - `on(blockX, blockY)`: The block ``blockX`` is stacked on top of ``blockY``.
 - `target-on-table(blockX)`: The goal is to have block ``blockX`` on the table
 - `-target-on(blockX, blockY`: The goal is to have block ``blockX`` stacked on top of ``blockY``.

Similarly, parameterized actions span the action space.

.. code-block:: lisp

      (rl-observable-action (name stack) (param-names r b1 b2) (param-types robot block block))
      (rl-observable-action (name pickup) (param-names r b) (param-types robot block))

This provides a sufficient model for the actions a robot may take.

 - `stack(robotX, blockX, blockY)`: Robot ``robotX`` stacks block ``blockX`` on top of ``blockY``.
 - `pickup(robotX, blockX)`: Robot ``robotX`` picks up block ``blockX``.

The parameters are grounded using the objects of the associated types and therfore form an observation space with 49 entries, as well as an action space with 20 entries.

.. code-block:: python

  # Full observation space is a discrete encoding of this vector
  ['on-table(block1)', 'on-table(block2)', 'on-table(block3)', 'on-table(block4)',
   'clear(block1)', 'clear(block2)', 'clear(block3)', 'clear(block4)',
   'can-hold(robot1)',
   'holding(robot1#block1)', 'holding(robot1#block2)', 'holding(robot1#block3)', 'holding(robot1#block4)',
   'on(block1#block1)', 'on(block1#block2)', 'on(block1#block3)', 'on(block1#block4)', 'on(block2#block1)', 'on(block2#block2)', 'on(block2#block3)', 'on(block2#block4)', 'on(block3#block1)', 'on(block3#block2)', 'on(block3#block3)', 'on(block3#block4)', 'on(block4#block1)', 'on(block4#block2)', 'on(block4#block3)', 'on(block4#block4)',
   'target-on(block1#block1)', 'target-on(block1#block2)', 'target-on(block1#block3)', 'target-on(block1#block4)', 'target-on(block2#block1)', 'target-on(block2#block2)', 'target-on(block2#block3)', 'target-on(block2#block4)', 'target-on(block3#block1)', 'target-on(block3#block2)', 'target-on(block3#block3)', 'target-on(block3#block4)', 'target-on(block4#block1)', 'target-on(block4#block2)', 'target-on(block4#block3)', 'target-on(block4#block4)',
    'target-on-table(block1)', 'target-on-table(block2)', 'target-on-table(block3)', 'target-on-table(block4)']

  # Full action space is a discrete encoding of this vector
  # plus the additional no-op action that is added automatically.
  ['stack(robot1#block1#block1)', 'stack(robot1#block1#block2)', 'stack(robot1#block1#block3)', 'stack(robot1#block1#block4)', 'stack(robot1#block2#block1)', 'stack(robot1#block2#block2)', 'stack(robot1#block2#block3)', 'stack(robot1#block2#block4)', 'stack(robot1#block3#block1)', 'stack(robot1#block3#block2)', 'stack(robot1#block3#block3)', 'stack(robot1#block3#block4)', 'stack(robot1#block4#block1)', 'stack(robot1#block4#block2)', 'stack(robot1#block4#block3)', 'stack(robot1#block4#block4)',
  'pickup(robot1#block1)', 'pickup(robot1#block2)', 'pickup(robot1#block3)', 'pickup(robot1#block4)']

Aside from the general observation space, the initial observations have to be specified:

.. code-block:: lisp

      (rl-observation (name clear) (param-values block1))
      (rl-observation (name clear) (param-values block2))
      (rl-observation (name clear) (param-values block3))
      (rl-observation (name clear) (param-values block4))
      (rl-observation (name on-table) (param-values block1))
      (rl-observation (name on-table) (param-values block2))
      (rl-observation (name on-table) (param-values block3))
      (rl-observation (name on-table) (param-values block4))
      (rl-observation (name target-on) (param-values block4 block3))
      (rl-observation (name target-on) (param-values block3 block2))
      (rl-observation (name target-on) (param-values block2 block1))
      (rl-observation (name target-on-table) (param-values block1))
      (rl-observation (name can-hold) (param-values robot1))
      (rl-observation (name clear) (param-values block1))
      (rl-observation (name clear) (param-values block2))
      (rl-observation (name clear) (param-values block3))
      (rl-observation (name clear) (param-values block4))

Hence, in the beginning, all blocks are placed on the table.

Next, the robot is initialized as acting entity, waiting to get a task assigned:

.. code-block:: lisp

      (rl-robot (name robot1) (waiting TRUE))

The setup is completed by asserting the cx-rl-node fact to notify the system.

.. code-block:: lisp

     (cx-rl-node (name ?*CX-RL-NODE-NAME*) (mode UNSET))
    )
  )


Blocksworld Step 2: Defining the Reset Procedure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The reset procedure is guided through the ``state`` slot of a fact of type  ``rl-reset-env``, which is asserted by the system whenever an environment reset is required (typically to start a new episode of training).
Per default it restores a backup of the fact base that is made after the ``cx-rl-node`` fact is asserted.

Users may define routins acting before (in state ``USER-CLEANUP``) and afterwards (in state ``USER-INIT``).
For this example, we use the default behavior and directly transition to from ``USER-CLEANUP`` to backup restoration (state ``LOAD-FACTS``) and from ``USER-INIT`` to ``DONE``, completing the reset.

.. code-block:: lisp

  (defrule reset-to-load-facts
    ?reset <- (rl-reset-env (state USER-CLEANUP))
    =>
    (modify ?reset (state LOAD-FACTS))
  )

  (defrule reset-to-done
    ?reset <- (rl-reset-env (state USER-INIT))
    =>
    (modify ?reset (state DONE))
  )


Blocksworld Step 3: Action Execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Action execution in both training and execution mode of the CXRLGym environment are centered around a fact of type ``rl-current-action-space``.

Whenever the current action space is in state `PENDING`, ``rl-action`` facts have to be asserted accordingly that reflect the current options for execution (utilizing the action masking feature of the provided policy).

.. code-block:: lisp

  (defrule rl-blocksworld-provide-action-stack
    (rl-current-action-space (state PENDING))
    (rl-robot (name ?robot) (waiting TRUE))
    (rl-observation (name holding) (param-values ?robot ?some-block))
    (rl-observation (name clear) (param-values ?other-block))
    (test (neq ?some-block ?other-block))
    =>
    (bind ?block1 (sub-string 6 6 ?some-block))
    (bind ?block2 (sub-string 6 6 ?other-block))
    (bind ?id (sym-cat "stack" ?block1 ?block2))
    (bind ?name (sym-cat "stack(" ?robot "#" ?some-block "#" ?other-block ")"))
    (assert (rl-action (id ?id) (name ?name)))
  )

  (defrule rl-blocksworld-provide-action-pickup
    (rl-current-action-space (state PENDING))
    (rl-robot (name ?robot) (waiting TRUE))
    (rl-observation (name can-hold) (param-values ?robot))
    (rl-observation (name on-table) (param-values ?some-block))
    (rl-observation (name clear) (param-values ?some-block))
    =>
    (bind ?block1 (sub-string 6 6 ?some-block))
    (bind ?id (sym-cat "pickup" (gensym*)))
    (bind ?name (sym-cat "pickup(" ?robot "#" ?some-block ")"))
    (assert (rl-action (id ?id) (name ?name)))
  )

  (defrule rl-blocksworld-actions-generation-done
    (declare (salience -1000))
    ?action-space <- (rl-current-action-space (state PENDING))
    =>
    (modify ?action-space (state DONE))
  )

The first rule ``rl-blocksworld-provide-action-stack`` asserts an ``rl-action`` for stacking a currently held block onto any other block that is currently clear of other blocks.

TODO: WE SHOULD NOT MANGLE RL-ACTION NAMES OURSELVES

.. code-block:: lisp

  (defrule rl-blocksworld-provide-action-stack
    (rl-current-action-space (state PENDING))
    (rl-robot (name ?robot) (waiting TRUE))
    (rl-observation (name holding) (param-values ?robot ?some-block))
    (rl-observation (name clear) (param-values ?other-block))
    (test (neq ?some-block ?other-block))
    =>
    (bind ?block1 (sub-string 6 6 ?some-block))
    (bind ?block2 (sub-string 6 6 ?other-block))
    (bind ?id (sym-cat "stack" ?block1 ?block2))
    (bind ?name (sym-cat "stack(" ?robot "#" ?some-block "#" ?other-block ")"))
    (assert (rl-action (id ?id) (name ?name)))
  )

Not
