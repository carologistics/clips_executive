Reinforcement Learning CLIPS Interfaces
#######################################

The reinforcement learning (RL) integration is provided in the form of the ``CXRLGym`` node, a ROS lifecycle node that exposes ROS interfaces to interact with RL models and environments.

The ``cx_rl_clips`` package provides a CLIPS-based interface for interacting with the RL Manager node in ROS 2. This allows asserting and monitoring CLIPS facts to:

- Configure RL environments.
- Query the current state of the environment.
- Execute actions and retrieve results.
- Reset or initialize agents for new episodes.

This reduces the need to interact with ROS messages or services directly from CLIPS.

CXRLGym Node
************

The ``CXRLGym`` node has the following responsibilities:

- Maintain the current environment state and RL model interface.
- Provide service interfaces for environment interactions:
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

To integrate the RL Manager with the |CX| framework, the following plugins are used:

- ``cx::ExecutivePlugin`` — orchestrates reasoning and execution between ROS and CLIPS.
- ``cx::RosMsgsPlugin`` — provides access to the RL Manager ROS services from CLIPS.
- ``cx::AmentIndexPlugin`` — resolves package paths to load environment or model files.


CXRLGym Interafaces
*******************

The ``CXRLGym`` node provides and consumes various ROS 2 services. The table below indicates whether it is a **service provider** or **client** for each endpoint.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Service / Topic
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

Example: Querying an environment state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # Initialize RL Manager interface
    rl_manager = RLManagerInterface(node)

    # Retrieve current environment state
    state = rl_manager.get_observation()
    print("Current state vector:", state)

Example: Executing an action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # Select action based on RL model
    response = rl_manager.exec_action_selection(request, response)
    print("Selected action ID:", response.actionid)
