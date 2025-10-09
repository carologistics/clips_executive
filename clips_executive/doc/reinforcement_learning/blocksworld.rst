.. _blocksworld_using_rl:

Blocksworld using Reinforcement Learning
#############################################

**Goal:** Creating a CLIPS agent solving a problem in a simple blocksworld domain and, in doing so, utilizing action selection using reinforcement learning.

**Tutorial level:** Intermediate

**Time:** 30 minutes

.. contents:: Contents
   :depth: 2
   :local:



Background
**********

This tutorial makes use of the RL plugin of the CLIPS Executive in combination with the CXRLGym environment and the Multi-Robot Maskable Action PPO agent which both are also situated in the ROS2 CLIPS-Executive. Furthermore, the pddl manager extention for the CLIPS Executive is used.


Prerequisites
*************

You will need to have the |CX| :doc:`installed <../getting_started/installation>` together with the pddl manager extension and sourced in your environment.


Action Selection using Reinforcement Learning
*********************************************

To communicate to the RL agent which actions can be selected at a certain step, the user must assert ``rl-action`` facts for every executable user action in the environment. These facts feature an ID and a name to map them to the action space (more on that later). Furthermore, the user can assign a worker robot to each rl-action by specifying it in the ``assigned-to`` slot. If this is not done (e.g. if all robots are able to execute all actions), the plugin does the robot assignment automactically based on which robot is currently idle for the longest time. For each selection step, all asserted rl-action facts are transmitted to the RL agent which then chooses one of the available action according to its policy, resulting in the selected fact's ``is-selected`` flag being set. Now the user can fetch this information and execute the selected action while all other rl-action facts are automatically retracted.

In training mode, when the execution of an action has finished, the user must set the ``is-finished`` flag. This leads to the reward and the current state being communicated back to RL agent, the rl-action fact being retracted and the next step being started.

In execution mode, the plugin requests an action selection whenever there is at least one unselected rl-action fact assigned to a waiting robot. Then these actions together with the current state of the environment are communicated to the RL agent which then predicts the next action, indicated by the set is-selected flag of the selected rl-action. After setting the is-finished flag, the rl-action fact is simply retracted as there is no need to give feedback to the RL agent.

To handle the given points for each rl-action, it is recommended to create a CLIPS-file dedicated to defining global point variables. This file must be loaded before the ``reinforcementlearning.clp`` and must contain the definitions for ``?*POINTS-EPISODE-END-SUCCESS`` and ``?*POINTS-EPISODE-END-FAILURE`` adding (or deducing) extra points for successful or failed episodes (can also be 0). Point mappings for the different actions can also be defined here.

Training is done over a certain number of episodes in the environment. To signal that an episode is finished, the user must assert the ``rl-episode-end`` fact whenever this is the case. Furthermore, the user can signal whether or not the episode was successful, i.e. a certain goal has been reached, and the ``success`` flag can be set accordingly. By default, if there are no rl-actions asserted when a previously selected one has finished, the episode is automatically ended and interpreted as unsuccessful. Therefore, it is important to use the saliences ``?*SALIENCE-ACTION-EXECUTABLE-CHECK*`` for every rule asserting rl-actions and ``?*SALIENCE-RL-EPISODE-END-SUCCESS*`` for rules asserting successful rl-episode-end facts.

Creating a Blocksworld Agent
****************************

1 Workspace Setup
~~~~~~~~~~~~~~~~~




2 Configuration
~~~~~~~~~~~~~~~

We need to create three configuration files in the ``params`` folder. The first file ``clips_env_manager.yaml`` features all plugins and files loaded in our CLIPS environment.

.. code-block:: yaml

  clips_manager:
    ros__parameters:
      environments: ["main"]
      main:
          plugins: ["executive",
                    "ament_index",
                    "config",
                    "ros_msgs",
                    "create_rl_env_state",
                    "exec_action_selection",
                    "get_observable_objects",
                    "get_observable_predicates",
                    "get_predefined_observables",
                    "get_action_list",
                    "get_action_list_robot",
                    "set_rl_mode",
                    "get_free_robot",
                    "action_selection",
                    "reset_cx",
                    "plan_temporal_action",
                    "files"]
          log_clips_to_file: true
          watch: ["facts", "rules"]
          redirect_stdout_to_debug: true

      ament_index:
        plugin: "cx::AmentIndexPlugin"

      config:
        plugin: "cx::ConfigPlugin"

      executive:
        plugin: "cx::ExecutivePlugin"
        publish_on_refresh: false
        assert_time: true
        refresh_rate: 10

      create_rl_env_state:
        plugin: "cx::CXCxRlInterfacesCreateRLEnvStatePlugin"

      exec_action_selection:
        plugin: "cx::CXCxRlInterfacesExecActionSelectionPlugin"

      get_observable_objects:
        plugin: "cx::CXCxRlInterfacesGetObservableObjectsPlugin"

      get_observable_predicates:
        plugin: "cx::CXCxRlInterfacesGetObservablePredicatesPlugin"

      get_predefined_observables:
        plugin: "cx::CXCxRlInterfacesGetPredefinedObservablesPlugin"

      get_action_list:
        plugin: "cx::CXCxRlInterfacesGetActionListPlugin"

      get_action_list_robot:
        plugin: "cx::CXCxRlInterfacesGetActionListRobotPlugin"

      set_rl_mode:
        plugin: "cx::CXCxRlInterfacesSetRLModePlugin"

      get_free_robot:
        plugin: "cx::CXCxRlInterfacesGetFreeRobotPlugin"

      action_selection:
        plugin: "cx::CXCxRlInterfacesActionSelectionPlugin"

      reset_cx:
        plugin: "cx::CXCxRlInterfacesResetCXPlugin"

      plan_temporal_action:
        plugin: "cx::CXExpertinoMsgsPlanTemporalPlugin"

      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["expertino", "cx_goal_reasoning", "cx_reinforcement_learning_plugin", "expertino_blocksworld"]
        load: [
          # Specific isolated concepts
          "clips/expertino/start-tasks/start-tasks.clp",
          # General utility stuff
          "clips/expertino/defglobals.clp",
          "clips/expertino/deftemplates.clp",
          # Startup
          "clips/expertino/pddl/saliences.clp",
          "clips/expertino_blocksworld/init-pddl.clp",
          # PDDL
          "clips/expertino/pddl/set-goals.clp",
          "clips/expertino/pddl/clear-goals.clp",
          "clips/expertino/pddl/objects.clp",
          "clips/expertino/pddl/fluents.clp",
          "clips/expertino_blocksworld/pddl/instances.clp",
          "clips/expertino/pddl/get-action-names.clp",
          "clips/expertino/pddl/get-fluents.clp",
          "clips/expertino/pddl/get-objects.clp",
          "clips/expertino/pddl/get-predicates.clp",
          "clips/expertino/pddl/numeric-fluents.clp",
          "clips/expertino/pddl/action-effect-apply.clp",
          "clips/expertino/action/action-precond-check.clp",
          # Point Mapping
          "clips/expertino_blocksworld/point-mapping.clp",
          # Reinforcement Learning Plugin
          "clips/rl_plugin/reinforcementlearning.clp",
          "clips/rl_plugin/reset-game.clp",
          "clips/rl_plugin/rl-utils.clp",
          "clips/rl_plugin/create-rl-env-state-srv.clp",
          "clips/rl_plugin/exec-action-selection-client.clp",
          "clips/rl_plugin/get-observable-objects-srv.clp",
          "clips/rl_plugin/get-observable-predicates-srv.clp",
          "clips/rl_plugin/get-predefined-observables-srv.clp",
          "clips/rl_plugin/get-action-list-srv.clp",
          "clips/rl_plugin/get-action-list-robot-srv.clp",
          "clips/rl_plugin/set-rl-mode-srv.clp",
          "clips/rl_plugin/get-free-robot-action.clp",
          "clips/rl_plugin/action-selection-action.clp",
          "clips/rl_plugin/reset-cx-action.clp",
          # Agent Files
          "clips/expertino_blocksworld/init.clp",
          "clips/expertino_blocksworld/reset.clp",
          "clips/expertino_blocksworld/action-executability.clp",
          "clips/expertino_blocksworld/action-execution.clp"
          ]
      ros_msgs:
        plugin: "cx::RosMsgsPlugin"




The second config is the ``training-config.yaml`` and features all settings for the reinforcement learning process. It is important, that the parameter ``number_of_robots`` matches the real number of worker robots in the environment to ensure no irregular behavior. By default it is configured to use training mode with a newly created agent which is saved as "BlocksworldAgent" in the tutorial agent package after training. The name can be changed using the parameter ``agent_name``. Existing agents can be trained for more episodes when enabling the ``retraining`` option. Change the parameter ``rl_mode`` to ``EXECUTION`` to enable the execution mode which uses an existing RL agent with the name as specified in the ``agent_name`` parameter.\

In the ``env/entrypoint`` setting, a custom environment class is specified which must define the action space of the RL agent and can be used for other custom operations like further logging.

When creating a new agent, several parameters can be set tp change its learning behavior. These follow largely the parameters of the `Stable Baselines3`_ implentation of the PPO algorithm. The ``wait_for_all_robots``parameter determines if the agent should wait until all robots have finished their actions before doing a policy update or if it does it directly when the nth step has completed.

.. code-block::yaml

  cxrl_node/blocksworld_rl_node:
    ros__parameters:
      package_dir: "expertino_ws/src/expertino-rcll/expertino_blocksworld/expertino_blocksworld/"
      agent_name: "BlocksworldAgent"
      rl_mode: "TRAINING"
      number_of_robots: 1

      training:
        retraining: false
        max_episodes: 100
        timesteps: 100000000

      env:
        entrypoint: "expertino_blocksworld.blocksworld_env:BlocksworldEnv"

      model:
        learning_rate: 0.0003
        gamma: 0.99
        gae_lambda: 0.95
        ent_coef: 0.0
        vf_coef: 0.5
        max_grad_norm: 0.5
        batch_size: 64
        n_steps: 10
        seed: 42
        verbose: 1
        wait_for_all_robots: false

The third config ``agent_config.yaml`` is used by the pddl manager extension to load the pddl domain and problem.

.. code-block::yaml
  # Configuration for the integration of the external pddl manager.
  pddl:
    # Name of the externally started pddl manager.
    manager_node: "/pddl_manager"
    # Directory relative to expertino share dir
    # used to lookup all files specified below.
    pddl_dir: "pddl"
    # Name of the main pddl instance.
    problem_instance: "blocksworld"
    # Initial pddl model split in domain and problem file.
    init_domain_file: "domain.pddl"
    init_problem_file: "problem.pddl"
    # For planning, a subset of actions is used, only actions from the provided domain file are used.
    planning_instance: "planning"
    planning_domain_file: "domain.pddl"



3 PDDL Domain
~~~~~~~~~~~~~

To start, we need to formalize our blocksworld domain in PDDL. For that, create a folder ``pddl`` with two files ``domain.pddl`` and ``problem.pddl``. In the first file, we define the overall nature of our domain (types, predicates, actions) and in the other we specify the initial state of our scenario. ``domain.pddl`` looks as follows:

.. code-block:: lisp

  (define (domain blocksworld)
  (:requirements :strips :typing)

  (:types
    block - object
      robot - object
  )

  (:predicates
    (clear ?a - block)
      (on-table ?a - block)
      (can-hold ?r - robot)
      (holding ?r - robot ?a - block)
      (on ?a - block ?b - block)
  )

  (:action pickup
    :parameters (?r - robot ?b - block)
    :precondition (and (clear ?b) (on-table ?b) (can-hold ?r))
    :effect (and (holding ?r ?b) (not (clear ?b)) (not (on-table ?b))
                (not (can-hold ?r))))

  (:action putdown
    :parameters  (?r - robot ?b - block)
    :precondition (and (holding ?r ?b))
    :effect (and (clear ?b) (can-hold ?r) (on-table ?b)
                (not (holding ?r ?b))))

  (:action stack
    :parameters  (?r - robot ?u - block ?l - block)
    :precondition (and  (clear ?l) (holding ?r ?u))
    :effect (and (can-hold ?r) (clear ?u) (on ?u ?l)
                (not (clear ?l)) (not (holding ?r ?u))))

  (:action unstack
    :parameters  (?r - robot ?u - block ?l - block)
    :precondition (and (on ?u ?l) (clear ?u) (can-hold ?r))
    :effect (and (holding ?r ?u) (clear ?l)
                (not (on ?u ?l)) (not (clear ?u)) (not (can-hold ?r))))
  )

This defines a simple domain with blocks that can be picked up and stacked by a robot. Now add the initial state to your ``problem.pddl``.

.. code-block:: lisp

  (define (problem blocksworld-problem)
    (:domain blocksworld)

    (:objects
        block1 block2 block3 block4 - block
        robot1 - robot
    )

    (:init
        (clear block1)
        (clear block2)
        (clear block3)
        (clear block4)

        (on-table block1)
        (on-table block2)
        (on-table block3)
        (on-table block4)

        (can-hold robot1)
    )

    (:goal (and
        (on block2 block1)
        (on block3 block2)
        (on block4 block3)
        )
    )
  )

In this case we have 4 block which we want to stack so that block1 is at the bottom and block4 is at the top.

4 CLIPS Files
~~~~~~~~~~~~~
Now we want to implement the main CLIPS agent. For that create a new folder ``clips`` where all CLIPS files are stored.

Init
++++

First, we define a ``init.clp`` where we load the domain, select which predicates and objects to observe and set the goal for a successful episode. Start with adding the init-load-domain rule to ensure the domain config is loaded.

.. code-block:: lisp

  (defrule init-load-domain
    (not (domain-loaded))
  =>
    (unwatch facts time)
    (unwatch rules time-retract)
    (bind ?share-dir (ament-index-get-package-share-directory "expertino_blocksworld"))
    (config-load (str-cat ?share-dir "/params/agent_config.yaml") "/")

    (assert (domain-loaded))
  )

Then we want to decide which objects and predicates in the domain we want to add to our automatic observation space generation. For the objects, we choose all of them. For the predicates we select all but the ``on-table`` predicate which we add to the observation space manually for demonstration purposes. Also, we make the robot visible for our RL plugin by asserting the ``rl-robot`` fact. For better readability, we create functions handling these selections.

.. code-block:: lisp

  (deffunction observe-predicates-except-on-table ()
    (do-for-all-facts ((?p pddl-predicate))
      (neq ?p:name on-table)
    (assert (rl-observable-predicate 	(name ?p:name)
                      (param-types ?p:param-types)
                      (param-names ?p:param-names)))
    )
  )

  (deffunction observe-all-objects ()
    (do-for-all-facts ((?o pddl-object))
      TRUE
    (assert (rl-observable-object	(name ?o:name)
                    (type ?o:type)))
    )
  )

  (deffunction predefine-observables ()
    (assert (rl-predefined-observable (name on-table) (params block1)))
    (assert (rl-predefined-observable (name on-table) (params block2)))
    (assert (rl-predefined-observable (name on-table) (params block3)))
  )

  (deffunction add-robot ()
    (assert (rl-robot (name robot1)))
  )

To ensure that our actions can be executed, we must assert a ``pddl-action`` fact for every action we want to be executable in our setting where the name and parameters of the fact must match the corresponding action defined in our ``domain.pddl``. Here we only allow blocks to be picked up and stacked without any unstacking procedure.

.. code-block:: lisp

  (deffunction generate-pddl-actions (?instance)
    (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block1)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block2)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block3)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block4)))

    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block2)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block3)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block4)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block1)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block3)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block4)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block1)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block2)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block4)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block1)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block2)))
    (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block3)))
  )

We also need to overwrite the function ``rl-generate-observations`` to ensure that ``rl-observation`` facts are asserted for every predicate that is true when the function is called. As we are using the pddl manager, these predicates are accessible via the ``pddl-fluent`` fact.

.. code-block:: lisp

  (deffunction rl-generate-observations ()
    (do-for-all-facts ((?pf pddl-fluent))
        TRUE
      (assert (rl-observation (name ?pf:name) (param-values ?pf:params)))
    )
  )

Before we can use these functions to set up our RL plugin, the predicates and objects from our pddl domain must be loaded into CLIPS by the pddl manager. For that, we make use of the get-predicates and get-objects services provided by the manager. When both service calls have concluded we are able to use our defined functions.

.. code-block:: lisp

  (defrule init-load-domain-facts
    (domain-loaded)
    (not (domain-facts-loaded))
    (startup-completed)
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (assert (pddl-get-predicates (instance (sym-cat ?instance-str))))
    (assert (pddl-get-objects (instance (sym-cat ?instance-str))))
  )

  (defrule init-load-domain-facts-done
    (not (domain-facts-loaded))
    (pddl-get-predicates (instance ?instance) (state DONE))
    (pddl-get-objects (instance ?instance) (state DONE))
    (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
    =>
    (predefine-observables)
    (observe-predicates-except-on-table)
    (observe-all-objects)
    (add-robot)
    (generate-pddl-actions ?instance)
    (assert (domain-facts-loaded))
  )

Finally, we want to define a goal state, which declares a successful outcome of our episode. As mentioned earlier, we we want to stack the block in ascending order with block1 being an the base.

.. code-block:: lisp

  (defrule domain-episode-finished-success
    (declare (salience ?*SALIENCE-RL-EPISODE-END-SUCCESS*))
    (not (rl-episode-end))
    (pddl-fluent (name on) (params block2 block1))
    (pddl-fluent (name on) (params block3 block2))
    (pddl-fluent (name on) (params block4 block3))
    =>
    (assert (rl-episode-end (success TRUE)))
  )

Because we do not consider any unstack actions, any deviation from the optimal stacking order will result in a tower of different ordering. When there is a tower of 4 blocks, we cannot perform any more actions which automactically results in the episode being ended and considered a failure.

Point Mapping
+++++++++++++

Create the file ``point-mapping.clp`` setting the reward given for the different actions and for the episode outcomes. Here we only give out a reward at the end of an episode.

.. code-block:: lisp

  (defglobal
    ?*POINTS-EPISODE-END-FAILURE* = -1
    ?*POINTS-EPISODE-END-SUCCESS* = 1
    ?*POINTS-ACTION-STACK* = 0
    ?*POINTS-ACTION-PICKUP* = 0
)

Resetting the PDDL Instance
+++++++++++++++++++++++++++

During training, the CLIPS environment is reset after every episode. To ensure that the domain is also reset, the pddl instance of the pddl manager needs to be reloaded at every reset. For that, create the file ``reset.clp`` with the following rule:

.. code-block:: lisp

  (defrule reset-reload-problem-instance
    (declare (salience ?*SALIENCE-RL-FIRST*))
    (reset-game-finished)
    (not (reset-instance))
    (pddl-manager (node ?node))
    (confval (path "/pddl/problem_instance") (value ?instance))
    (confval (path "/pddl/pddl_dir") (value ?dir))
    (confval (path "/pddl/init_domain_file") (value ?domain))
    (confval (path "/pddl/init_problem_file") (value ?problem))
    ?ins <- (pddl-instance)
    =>
    (retract ?ins)
    (assert (pddl-instance (name (sym-cat ?instance)) (domain ?domain) (problem ?problem) (directory ?dir) (state PENDING)))
    (assert (reset-instance))
  )

Action Executability
++++++++++++++++++++

Create the file ``action-executability.clp`` where actions are checked for their preconditions defined in the ``domain.pddl`` using the action-predcondition service of the pddl manager. If all preconditions of an action are satisfied, a ``rl-action`` fact is asserted, making this action available for the next action selection. When all actions have been checked (i.e. there is no action being checked anymore), the ``rl-executability-check`` fact of the RL plugin is set to CHECKED, allowing the next action selection to be executed.

.. code-block:: lisp

  (defrule check-action
      (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
      (rl-executability-check (state CHECKING))
      (pddl-action (id ?action-id) (name ?action))
      (not (pddl-action-precondition (id ?action-id)))
      (not (rl-action (id ?action-id) (is-finished TRUE)))
      (not (rl-action (name ?action) (is-selected TRUE)))
      =>
      (assert (pddl-action-precondition (id ?action-id)))
  )

  (defrule executable-action
      (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
      (rl-executability-check (state CHECKING))
      (pddl-action-precondition (id ?action-id) (state PRECONDITION-SAT))
      (pddl-action (id ?action-id) (name ?name) (params $?params))
      =>
      (assert (rl-action (id ?action-id) (name (sym-cat ?name "#" (create-slot-value-string $?params))) (points 0)))
  )

  (defrule executability-check-finished
      (declare (salience (- ?*SALIENCE-ACTION-EXECUTABLE-CHECK* 1)))
      ?ec <- (rl-executability-check (state CHECKING))
      (not (pddl-action-precondition (state PENDING|CHECK-PRECONDITION)))
      =>
      (modify ?ec (state CHECKED))
      (do-for-all-facts ((?ap pddl-action-precondition))
          (retract ?ap)
      )
  )

Execution of Actions
++++++++++++++++++++

Create a new file ``action-execution.clp``. We can detect if an rl-action has been selected by looking at its ``is-selected`` slot. If it is set to TRUE, the matching pddl-action needs to be executed. This is done by calling the action-apply-effect service of the pddl manager.

.. code-block:: lisp

  (defrule execute-action
    (rl-action (id ?action-id) (is-selected TRUE))
    (pddl-action (id ?action-id))
    (not (pddl-action-apply-effect (action ?action-id)))
    =>
    (assert (pddl-action-apply-effect (action ?action-id)))
  )

When the action effect has been applied, we want to signal back to the RL plugin that the action has finished by setting the ``is-finished``slot to TRUE in the corresponding rl-action fact.

.. code-block:: lisp

  (defrule execute-action-done
    (pddl-action-apply-effect (action ?action-id) (state DONE))
    ?ra <- (rl-action (id ?action-id) (is-selected TRUE))
    =>
    (modify ?ra (is-finished TRUE))
  )

5 Custom Python Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~

A custom environment is created (see ``cxrl_blocksworld/blocksworld_env.py``) which inherits from the ``CXRLGym`` class of the ``cx_reinforcement_learning`` package. In the custom CXRLGym-environment, the ``generate_action_space`` function is overwritten to list all possible rl-action names. In this case it is a combination of the goal class and its parameters. Other gym-functions can be extended to add custom functionality, here additional logging has been added to the ``step`` and ``reset`` function.

.. code-block:: python

  from cxrl_gym.cxrl_gym import CXRLGym
  from rclpy.node import Node
  import rclpy


  class BlocksworldEnv(CXRLGym):
      def __init__(self, node: Node, mode: str, number_robots: int):
          self.reward_in_episode = 0
          super().__init__(node, mode, number_robots)

      def step(self, action):
          with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
              f.write(f"{self.action_dict[action]} \n")
          state, reward, done, truncated, info = super().step(action)
          self.reward_in_episode += reward
          return state, reward, done, truncated, info

      def reset(self, seed: int = None, options: dict[str, any] = None):
          with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
              f.write(f"{self.reward_in_episode} \n")
          self.reward_in_episode = 0
          return super().reset(seed=seed)

      def generate_action_space(self):
          self.node.get_logger().info("Generating action space...")
          action_space =  ["stack#robot1#block1#block2",
                          "stack#robot1#block1#block3",
                          "stack#robot1#block1#block4",
                          "stack#robot1#block2#block1",
                          "stack#robot1#block2#block3",
                          "stack#robot1#block2#block4",
                          "stack#robot1#block3#block1",
                          "stack#robot1#block3#block2",
                          "stack#robot1#block3#block4",
                          "stack#robot1#block4#block1",
                          "stack#robot1#block4#block2",
                          "stack#robot1#block4#block3",
                          "pickup#robot1#block1",
                          "pickup#robot1#block2",
                          "pickup#robot1#block3",
                          "pickup#robot1#block4"
                          ]
          return action_space

      def render(self):
          pass
