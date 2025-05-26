.. _usage_action_selection_using_rl:

Action Selection using Reinforcement Learning
#############################################

Source code on .

This plugin provides the ability to train an RL agent's policy to decide on which action to pursue next in the CLIPS Executive using reinforcement learning. To that end, it provides two modes, one for training the RL agent (Mode: ``TRAINING``) and one for predicting the next action according to the trained policy when requested by the CX (Mode: ``EXECUTION``).

Installation
************

The cxrl plugin is situated in another repository which can be installed as follows:

.. code-block:: bash

  mkdir -p ~/cxrl_ws/src
  cd ~/cxrl_ws/src
  git clone https://github.com/Frankpier/cxrl.git
  cd ..
  colcon build --symlink-install
  source ~/cxrl_ws/install/setup.bash

Configuration
*************

:`package_dir`:

  ====== =======
  Type   Default
  ------ -------
  string []
  ====== =======

  Description
    Specify the path to your ``cx_reinforcement_learning`` installation.


:`agent_name`:

  ====== =======
  Type   Default
  ------ -------
  string []
  ====== =======

  Description
    Specify the name of RL agent using which it is saved after training and loaded in case of further training or in execution mode.


:`rl_mode`:

  ====== =======
  Type   Default
  ------ -------
  string []
  ====== =======

  Description
    Set the mode of the plugin, available modes are "TRAINING" and "EXECUTION".


:`number_of_robots`:

  ======= =======
  Type    Default
  ------- -------
  integer []
  ======= =======

  Description
    Set the mode of the plugin, available modes are "TRAINING" and "EXECUTION".


:`retraining`:

  ======= =======
  Type    Default
  ------- -------
  boolean []
  ======= =======

  Description
    Specify if in case of "TRAINING" mode, a new RL agent should be created (``false``) or an existing agent should be loaded and trained further(``true``).


:`max_episodes`:

  ======= =======
  Type    Default
  ------- -------
  integer []
  ======= =======

  Description
    Set the maximum number of environment episodes the RL agent should train for.


:`timesteps`:

  ======= =======
  Type    Default
  ------- -------
  integer []
  ======= =======

  Description
    Set the maximum numbeer of steps the RL agent should do in the environment.


:`entrypoint`:

  ====== =======
  Type   Default
  ------ -------
  string []
  ====== =======

  Description
    Specify the custom cxrl_gym environment created for your own domain. 

Features
********

To communicate to the RL agent which actions can be selected at a certain step, the user must assert ``rl-action`` facts for every executable user action in the environment. These facts feature an ID and a name to map them to the action space (more on that later). Furthermore, the user can assign a worker robot to each rl-action by specifying it in the ``assigned-to`` slot. If this is not done (e.g. if all robots are able to execute all actions), the plugin does the robot assignment automactically based on which robot is currently idle for the longest time. For each selection step, all asserted rl-action facts are transmitted to the RL agent which then chooses one of the available action according to its policy, resulting in the selected fact's ``is-selected`` flag being set. Now the user can fetch this information and execute the selected action while all other rl-action facts are automatically retracted.

In training mode, when the execution of an action has finished, the user must set the ``is-finished`` flag. This leads to the reward and the current state being communicated back to RL agent, the rl-action fact being retracted and the next step being started.

In execution mode, the plugin requests an action selection whenever there is at least one unselected rl-action fact assigned to a waiting robot. Then these actions together with the current state of the environment are communicated to the RL agent which then predicts the next action, indicated by the set is-selected flag of the selected rl-action. After setting the is-finished flag, the rl-action fact is simply retracted as there is no need to give feedback to the RL agent.

To handle the given points for each rl-action, it is recommended to create a CLIPS-file dedicated to defining global point variables. This file must be loaded before the ``reinforcementlearning.clp`` and must contain the definitions for ``?*POINTS-EPISODE-END-SUCCESS`` and ``?*POINTS-EPISODE-END-FAILURE`` adding (or deducing) extra points for successful or failed episodes (can also be 0). Point mappings for the different actions can also be defined here.

Training is done over a certain number of episodes in the environment. To signal that an episode is finished, the user must assert the ``rl-episode-end`` fact whenever this is the case. Furthermore, the user can signal whether or not the episode was successful, i.e. a certain goal has been reached, and the ``success`` flag can be set accordingly. By default, if there are no rl-actions asserted when a previously selected one has finished, the episode is automatically ended and interpreted as unsuccessful. Therefore, it is important to use the saliences ``?*SALIENCE-ACTION-EXECUTABLE-CHECK*`` for every rule asserting rl-actions and ``?*SALIENCE-RL-EPISODE-END-SUCCESS*`` for rules asserting successful rl-episode-end facts.

Facts
~~~~~

.. code-block:: lisp

  ;asserted by the user whenever a corresponding user action is executable
  (deftemplate rl-action
      (slot id (type SYMBOL))
      (slot name (type SYMBOL))
      (slot is-selected   (type SYMBOL)
                          (allowed-values TRUE FALSE)
                          (default FALSE))
      (slot is-finished   (type SYMBOL)
                          (allowed-values TRUE FALSE)
                          (default FALSE))
      (slot assigned-to   (type SYMBOL) 
                          (default nil))
      (slot points    (type INTEGER) 
                      (default 0))
  )

  ;asserted by the user if the current episode has finished
  (deftemplate rl-episode-end
      (slot success   (type SYMBOL)
                      (allowed-values TRUE FALSE)
                      (default TRUE))
  )

  ;asserted by the plugin, corresponds to a single selection process during training mode
  (deftemplate rl-action-selection
      (slot uuid (type STRING))
      (slot actionid (type SYMBOL))
      (slot is-finished   (type SYMBOL)
                          (allowed-values TRUE FALSE)
                          (default FALSE))
      (slot reward    (type INTEGER)
                      (default 0))
      (slot done  (type SYMBOL)
                  (allowed-values TRUE FALSE)
                  (default FALSE))
  )

  ;asserted by the plugin, corresponds to a single action selection during execution mode
  (deftemplate rl-action-selection-exec
      (slot actionid (type SYMBOL))
  )

  ;asserted by the plugin, shows the current mode
  (deftemplate rl-mode
      (slot mode  (type SYMBOL)
                  (allowed-values TRAINING EXECUTION))
  )

  ;asserted by the plugin whenever a robot is not assigned to a selected rl-action
  (deftemplate robot-waiting
      (slot robot (type SYMBOL))
  )

Usage Example
*************

A minimal usage example is provided in the cxrl-blocksworld repository, where a CLIPS Executive agent is tasked to stack three blocks on top of each other in a certain order. To run it, first navigate to ``src/cxrl_blocksworld/params/training-config.yaml`` and use ``cxrl_node/blocksworld_rl_node/ros_parameters/package_dir`` to specify the path to the location of the ``cx_reinforcement_learning`` files (the cx_reinforcement_learning workspace needs to be sourced as well). Then run:

.. code-block:: bash
  
  ros2 launch cxrl_blocksworld agent.launch.py

Configuration
~~~~~~~~~~~~~

There are two configuration files, one for the Ros2 CLIPS Executive and one for CXRL. The first one sets up the communication interfaces and loads all necessary CLIPS files. It can be found at ``cxrl-blocksworld/params/agent.yaml``:

.. code-block:: yaml
  
  clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
        plugins: ["executive",
                  "clips_pddl_parser",
                  "ament_index",
                  "config",
                  "create_rl_env_state",
                  "exec_action_selection",
                  "get_domain_objects",
                  "get_domain_predicates",
                  "get_action_list",
                  "get_action_list_robot",
                  "set_rl_mode",
                  "get_free_robot",
                  "action_selection",
                  "reset_cx",
                  "files"]
        log_clips_to_file: true
        watch: ["facts", "rules"]
        redirect_stdout_to_debug: true

    ament_index:
      plugin: "cx::AmentIndexPlugin"
    
    config:
      plugin: "cx::ConfigPlugin"

    clips_pddl_parser:
      plugin: "cx::PddlParserPlugin"

    executive:
      plugin: "cx::ExecutivePlugin"
      publish_on_refresh: false
      assert_time: true
      refresh_rate: 10
    
    create_rl_env_state:
      plugin: "cx::CXCxRlInterfacesCreateRLEnvStatePlugin"

    exec_action_selection:
      plugin: "cx::CXCxRlInterfacesExecActionSelectionPlugin"

    get_domain_objects:
      plugin: "cx::CXCxRlInterfacesGetDomainObjectsPlugin"
    
    get_domain_predicates:
      plugin: "cx::CXCxRlInterfacesGetDomainPredicatesPlugin"

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

    files:
      plugin: "cx::FileLoadPlugin"
      pkg_share_dirs: ["cxrl_blocksworld", "cx_goal_reasoning", "cx_reinforcement_learning_plugin"]
      load: [
        "clips/blocksworld-agent/point-mapping.clp",
        "clips/cx_goal_reasoning/plan.clp",
        "clips/cx_goal_reasoning/goal.clp",
        "clips/cx_goal_reasoning/goal-tree.clp",
        "clips/cx_goal_reasoning/pddl-action.clp",
        "clips/cx_goal_reasoning/action-selection/sequential.clp",
        "clips/rl_plugin/reinforcementlearning.clp",
        "clips/rl_plugin/reset-game.clp",
        "clips/rl_plugin/rl-utils.clp",
        "clips/rl_plugin/create-rl-env-state-srv.clp",
        "clips/rl_plugin/exec-action-selection-client.clp",
        "clips/rl_plugin/get-domain-objects-srv.clp",
        "clips/rl_plugin/get-domain-predicates-srv.clp",
        "clips/rl_plugin/get-action-list-srv.clp",
        "clips/rl_plugin/get-action-list-robot-srv.clp",
        "clips/rl_plugin/set-rl-mode-srv.clp",
        "clips/rl_plugin/get-free-robot-action.clp",
        "clips/rl_plugin/action-selection-action.clp",
        "clips/rl_plugin/reset-cx-action.clp",
        "clips/blocksworld-agent/init.clp",
        "clips/blocksworld-agent/goal-production.clp",
        "clips/blocksworld-agent/goal-expansion.clp",
        "clips/blocksworld-agent/goal-executability.clp"
        ]

The second config can be found at ``cxrl-blocksworld/params/training-config.yaml`` and features als settings for the reinforcement learning process. It is important, that the parameter ``number_of_robots`` matches the real number of worker robots in the environment to ensure no irregular behavior. By default it is configured to use training mode with a newly created agent which is saved as "BlocksworldAgent" in the cx_reinforcement_learning package after training. The name can be changed using the parameter ``agent_name``. Existing agents can be trained for more episodes when enabling the ``retraining`` option. Change the parameter ``rl_mode`` to ``EXECUTION`` to enable the execution mode which uses an existing RL agent with the name as specified in the ``agent_name`` parameter.\

In the ``env/entrypoint`` setting, a custom environment class is specified which must define the action space of the RL agent and can be used for other custom operations like further logging.

When creating a new agent, several parameters can be set tp change its learning behavior. These follow largely the parameters of the `Stable Baselines3`_ implentation of the PPO algorithm. The ``wait_for_all_robots``parameter determines if the agent should wait until all robots have finished their actions before doing a policy update or if it does it directly when the nth step has completed.

.. code-bloc::yaml

  cxrl_node/blocksworld_rl_node:
    ros__parameters:
      package_dir: "ros2/clips_executive_ws/src/ros2-clips-executive/cx_reinforcement_learning/cx_reinforcement_learning"    
      agent_name: "BlocksworldAgent"
      rl_mode: "TRAINING"
      number_of_robots: 1    
      
      training:
        retraining: false
        max_episodes: 100
        timesteps: 100000000    
      
      env:
        entrypoint: "cxrl_blocksworld.blocksworld_env:BlocksworldEnv"    
      
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


Code
~~~~

Custom environment
++++++++++++++++++

A custom environment is created (see ``cxrl_blocksworld/blocksworld_env.py``) which inherits from the ``CXRLGym``class of the ``cx_reinforcement_learning`` package. In the custom CXRLGym-environment, the ``generate_action_space`` function is overwritten to list all possible rl-action names. In this case it is a combination of the goal class and its parameters. Other gym-functions can be extended to add custom functionality, here additional logging has been added to the ``step`` and ``reset`` function.

.. code-block:: python
  
  from cx_reinforcement_learning.cxrl_gym import CXRLGym
  from rclpy.node import Node
  import rclpy


  class BlocksworldEnv(CXRLGym):
      def __init__(self, node: Node):
          self.reward_in_episode = 0
          super().__init__(node)

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
          action_space =  ["STACK#upper#block1#lower#block2",
                          "STACK#upper#block1#lower#block3",
                          "STACK#upper#block2#lower#block1",
                          "STACK#upper#block2#lower#block3",
                          "STACK#upper#block3#lower#block1",
                          "STACK#upper#block3#lower#block2"
                          ]       
          return action_space

      def render(self):
          pass

Episode Goal
++++++++++++

In this blocksworld environment, the agent is tasked to stack ``block2`` on ``block1`` and ``block3`` on ``block2`` (see ``init.clp``):

.. code-block:: lisp

  (defrule domain-episode-finished-success
    (declare (salience ?*SALIENCE-RL-EPISODE-END-SUCCESS*))
    (not (rl-episode-end))
    (domain-fact (name on) (param-values block2 block1))
    (domain-fact (name on) (param-values block3 block2))
    =>
    (assert (rl-episode-end (success TRUE)))
  ) 

Point Mapping
+++++++++++++

As shown in ``point-mapping.clp``, there are no points given for stacking blocks but only for the successful episodes:

.. code-block:: lisp

  (defglobal
    ?*POINTS-EPISODE-END-FAILURE* = -1
    ?*POINTS-EPISODE-END-SUCCESS* = 1
    ?*POINTS-GOAL-STACK* = 0
  )

Generation of RL-Actions
++++++++++++++++++++++++

In the ``goal-executability.clp``, STACK goals (consisting of a robot picking up a block and putting it on another) are checked on their executability, i.e. if the block to be stacked is lying on the table and the block to be stacked on is free on top. If this is the case, an RL-Action fact is asserted:

.. code-block:: lisp

  (defrule goal-executable-stack
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    ?g <-   (goal   (class STACK) (id ?goalid)
                    (mode FORMULATED) (params upper ?upper lower ?lower))
    (not    (goal   (class STACK)
                    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
    (domain-fact (name clear) (param-values ?lower))
    (domain-fact (name clear) (param-values ?upper))
    (domain-fact (name on-table) (param-values ?upper))
    =>
    (printout t "Goal STACK executable" crlf)
    (assert (rl-action (id ?goalid) (name (sym-cat "STACK#upper#" ?upper "#lower#" ?lower)) (points ?*POINTS-GOAL-STACK*)))
  )

Detecting Selections
++++++++++++++++++++

As seen in the ``goal-expansion.clp``, when the RL agent has selected an RL-Action, the CLIPS agent selects the corresponding STACK goal:

.. code-block:: lisp

  (defrule goal-reasoner-select 
	(rl-action (id ?aid) (is-selected TRUE))
	?g <- (goal (id ?aid) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
  )

Communicating finished Actions
++++++++++++++++++++++++++++++

Again in ``goal-expansion.clp``, when the execution of a goal has finished, the ``is-finished`` flag of the rl-action fact is set so that the reward is sent back to the RL agent:

.. code-block:: lisp

  (defrule goal-reasoner-completed
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome&~UNKNOWN))
	?a <- (rl-action (id ?goal-id) (is-selected TRUE))
	=>
    (modify ?a (is-finished TRUE))

	(printout t "Goal '" ?goal-id "' has been completed, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
		(retract ?p)
	)
	(retract ?g)
  )