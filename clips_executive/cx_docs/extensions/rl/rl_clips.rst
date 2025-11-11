Reinforcement Learning CLIPS Interfaces
#######################################

The Reinforcement Learning (RL) integration is provided through the CXRLGym node.
It provides a Gym environemnt that utilizes ROS interaces to customize training and execution, which can be called through the |CX| plugins that handle ROS communication.


This makes it possible to:

* Train reinforcement learning agents using ROS
* Integrate symbolic reasoning and learning-based control by combining it with CLIPS
* Evaluate trained models directly on real or simulated robots

The CXRLGym node is providing a sceleton, which custom RL policies can be based on in order to provide a uniform interface to the ROS ecosystem and the |CX| in particular.

In order to reduce the manual overhead, the ``cx_rl_clips`` package provides a CLIPS-based interface for interacting with the PDDL Manager node in ROS 2.
This allows to interact with the CXRLGym manager by simply asserting and monitoring CLIPS facts, without the need to do direct ROS communication (e.g., populating ROS messages or waiting for service feedback).


CXRLGym
*******

The CXRLGym node exposes a Gymnasium-compatible environment that interacts with ROS 2 services and actions.

The ``CXRLGym`` node has the following responsibilities:

- Maintain the current environment state and RL model interface.
- Provide ROS interfaces for environment interactions:
  - Create environment state.
  - Execute actions using the RL model.
  - Reset the environment or agent.
  - Query available robots and objects.
  - Retrieve observable predicates.
- Convert facts from CLIPS into state vectors for the RL model.
- Manage executable actions, including action masks for valid actions.
- Support multi-robot environments by tracking robot availability.

Required |CX| Plugins
*********************

In order to integrate the CXRLGym with the |CX|, the following plugins are needed:

- ``cx::ExecutivePlugin`` — manages the overall reasoning and control flow, interleaving ROS feedback with CLIPS reasoning.
- ``cx::RosMsgsPlugin`` provides access to ROS interfaces of the PDDL manager from within CLIPS.
- ``cx::AmentIndexPlugin`` resolves package paths via ``ament_index``. While not required, it is very useful in order to load agents.

Also, as the current configuration is compatible with ROS 2 jazzy, action client and service introspection is not supported, hence the following plugins are needed:

- ``cx::CXCxPddlMsgsPlanTemporalPlugin``
- ``cx::CXCxPddlMsgsTimedPlanActionPlugin``
- ``cx::CXCxRlInterfacesGetFreeRobotPlugin``
- ``cx::CXCxRlInterfacesExecActionSelectionPlugin``
 - ``cx::CXCxRlInterfacesResetCXPlugin``


CXRLGym Interafaces
*******************

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
     - Requests executable actions filtered by robot.
   * - ``/get_observable_objects`` (client)
     - Requests observable objects in the environment for a given type.
   * - ``/get_observable_predicates`` (client)
     - Requests predicates and their parameters observable in the environment.
   * - ``/get_predefined_observables`` (client)
     - Requests predefined observables available in the environment.
   * - ``/reset_cx`` (client)
     - Requests to reset the environment or agent and receives confirmation.
   * - ``/get_free_robot`` (client)
     - Requests an available robot in multi-robot scenarios.

Workflow Overview
*****************

The RL Manager provides a structured workflow for interacting with environments:

1. **Initialize the environment**: Create the environment state and optionally configure mode or initial parameters.
2. **Observe the state**: Retrieve observations or convert facts to state vectors for RL models.
3. **Execute actions**: Select actions via the RL model and apply them to the environment.
4. **Observe results**: Retrieve updated state, check available actions, and track rewards.
5. **Reset environments**: Start new episodes for training or evaluation.

Class Overview
**************

``CXRLGym(node: rclpy.node.Node, mode: str, number_robots: int)``
   A subclass of :class:`gymnasium.Env` that manages RL interaction over ROS 2.

Core methods:

+------------------------+----------------------------------------------------+
| Method                 | Description                                        |
+========================+====================================================+
| ``step(action)``       | Executes one environment step via ROS 2 actions    |
+------------------------+----------------------------------------------------+
| ``reset()``            | Resets the environment and returns initial obs.    |
+------------------------+----------------------------------------------------+
| ``set_rl_model(model)``| Registers a trained RL model and exposes it via    |
|                        | ROS 2                                              |
+------------------------+----------------------------------------------------+
| ``action_masks()``     | Returns currently valid actions for a robot        |
+------------------------+----------------------------------------------------+
| ``get_observation()``  | Builds observation vector from symbolic facts      |
+------------------------+----------------------------------------------------+

The observation space is a continuous ``Box(0, 1, (n_obs,))``,
and the action space is discrete ``Discrete(n_actions)``.

CLIPS Integration
*****************

TODO
