RL Node
#######

Reinforcement learning in this extension is enabled via a ROS node, which manages a RL environment and a RL algorithm.

The core is a `Gymnasium`_ environment derived fro mthe  CXRLGym class, which exchanges all information regarding observations and actions via ROS and hence provides a generic interface.

Along with an environment we provide a suitable RL algorithm and bundle both in the ``CXRLNode``.

CXRLNode
********

The CXRLNode is a node to dynamically load a CXRLGym environment and to manage training and execution of a multi-robot reinforcement learning agent using a maskable PPO algorithm.

The node supports both training and execution modes and interfaces with ROS 2 services and actions through a dynamically loaded environment.
Depending on the configured RL mode it performs the following tasks:

* **Training**

  * Create or load a PPO agent
  * Train for a specified number of timesteps or episodes
  * Save checkpoints and the final trained model

* **Execution**

  * Load an existing trained agent
  * Execute the policy until shutdown is requested


CXRLGym
*******

CXRLGym provides a Gymnasium-compatible reinforcement learning environment that integrates tightly with ROS 2.
It supports both single-robot and multi-robot scenarios, including settings where a single shared policy is trained and deployed across multiple robots.


Observation Space
-----------------

The observation space encodes the symbolic world state as a fixed-size numerical feature vector.

* The symbolic state is defined using predicates and objects.
* Each feature corresponds to the truth value of a grounded fluent and is represented as a binary value in the interval :math:`[0, 1]`.
* The observation vector is exposed as a Gymnasium ``Box`` space with shape ``(n_obs,)``.

Two mechanisms for defining observables are supported:

* **Automatically grounded predicates**

  Predicates are grounded over all compatible object combinations, yielding a complete propositional encoding of the symbolic state. Each grounded predicate corresponds to one feature in the observation vector.
  This is realized using the ROS services GetObservableObjects and GetObservablePredicates.

* **Predefined observables**

  In addition to automatic grounding, predefined observables with fixed predicate arguments can be specified explicitly. These observables do not span the full object space and allow the definition of domain-specific or abstracted features.
  This is realized using the ROS service GetPredefinedPredicates.

Both types of observables are combined into a single observation vector, enabling a trade-off between representational completeness and compactness.

Action Space
------------

The action space is represented as a discrete set of action names and is exposed as a Gymnasium ``Discrete(n_actions)`` space. Each discrete value corresponds to a named high-level action.

.. todo::

  THIS IS STILL MISSING

The complete action set is initialized via the ROS 2 service ``GetActionSpace``, which provides the list of all actions supported by the system.

CXRLGym is designed to support multi-robot scenarios. In this setting, actions are assumed to be assignable to individual robots. Action masking is used to indicate which actions are executable for a given robot at each decision step. The set of currently executable actions for a robot is obtained through the ``GetActionListRobot`` service.

Step Function
-------------

The environment step proceeds as follows:

* The selected action is checked for executability with respect to the currently assigned robot.
* If the action is not executable:

  * The current observation is returned.
  * The reward is computed.
  * The episode termination condition is evaluated via the ``GetEpisodeEnd`` service.

* If the action is executable:

  * The action is dispatched using the ``ActionSelection`` ROS action interface.
  * After execution, the reward, updated observation, and termination status are retrieved.

CXRLGym Interfaces
------------------

The ``CXRLGym`` node provides and consumes various ROS 2 services. The table below indicates whether it is a **service provider** or **client** for each endpoint.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Service / Action
     - Description
   * - ``/exec_action_selection`` (service provider)
     - Executes an action selection step using the RL model and returns the selected action.
   * - ``/create_rl_env_state`` (client)
     - Requests the current environment state as a serialized string of facts.
   * - ``/set_rl_mode`` (client)
     - Requests to set the RL mode (e.g., train, test) and receives confirmation.
   * - ``/get_action_list_executable`` (client)
     - Requests all actions currently executable in the environment.
   * - ``/get_action_list_executable_for_robot`` (client)
     - Requests executable actions for a robot.
   * - ``/get_observable_objects`` (client)
     - Requests observable objects in the environment for a given type.
   * - ``/get_observable_predicates`` (client)
     - Requests predicates and their parameters observable in the environment.
   * - ``/get_predefined_observables`` (client)
     - Requests predefined observables available in the environment.
   * - ``/reset_cx`` (action client)
     - Reset the environment or agent and receives confirmation.
   * - ``/get_free_robot`` (action client)
     - Determine an available robot in multi-robot scenarios.
   * - ``/action_selection`` (action client)
     - Perform the execution of the selected action.


Multi-Robot Maskable Proximal Policy Optimization
*************************************************

MultiRobotMaskablePPO is an extension of the `MaskableActorCriticPolicy`_ from `sb3_contrib`_ designed for multi-robot reinforcement learning scenarios.

The algorithm enables multiple robots to act concurrently while sharing a single policy, combining invalid action masking with parallel rollout collection.

Key Features
------------

* Shared policy across multiple robots
* Concurrent action execution using parallel threads
* Support for invalid action masking via ``MaskableActorCriticPolicy``
* Time-based and step-based rollout collection modes
* Custom rollout buffers with action mask support

Multi-Robot Rollout Collection
------------------------------

Rollouts are collected concurrently by spawning up to ``n_robots`` worker threads. Each thread executes a single environment step using the current policy and records the resulting transition.

Depending on configuration, rollout collection terminates when either a fixed number of steps is reached or a predefined time budget expires. An optional synchronization barrier ensures that all robot threads complete before advantage computation.

Action Masking
--------------

Invalid action masking is supported during both training and inference. At each decision step, the environment provides a binary action mask indicating executable actions. The policy restricts action selection accordingly, ensuring that only valid actions are sampled.

Custom Rollout Buffers
----------------------

The algorithm employs specialized rollout buffers that store action masks alongside observations, actions, rewards, and value estimates. This allows masked actions to be correctly handled during policy optimization and advantage computation.
