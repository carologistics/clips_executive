# Copyright (c) 2025-2026 Carologistics
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
CXRLGym: ROS 2 Reinforcement Learning Environment.

This module defines the :class:`CXRLGym` class — a ROS 2-integrated
reinforcement learning environment following the Gymnasium API.

It bridges ROS 2 cognitive execution (CX) systems and reinforcement
learning frameworks such as Stable Baselines3. Agents interact through
Gym-compatible `step()` and `reset()` interfaces, while environment
state and actions are managed via ROS 2 services and actions.
"""


import ast
from functools import partial
from itertools import product
import time

from cx_rl_interfaces.action import ActionSelection, GetFreeRobot, ResetEnv
from cx_rl_interfaces.srv import (
    EndTraining,
    ExecActionSelection,
    GetActionList,
    GetActionListRobot,
    GetEnvState,
    GetEpisodeEnd,
    GetObservableActions,
    GetObservableObjects,
    GetObservablePredicates,
    GetPredefinedActions,
    GetPredefinedObservables,
    SetRLMode,
)
from gymnasium import Env
from gymnasium.spaces import Box, Discrete
import numpy as np
import numpy.typing as npt
import pandas as pd
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future


class CXRLGym(Env):
    """
    ROS 2 integrated Gymnasium environment for reinforcement learning.

    This environment dynamically constructs its observation and action spaces
    from ROS 2 services and executes symbolic robot actions through ROS 2 actions.

    Parameters
    ----------
      node (rclpy.node.Node): ROS 2 node used to create service and action clients.
      mode (str): Reinforcement learning mode (e.g., "train" or "eval").
      number_robots (int): Number of managed robots in the environment.

    """

    def __init__(self, node: Node, mode: str, number_robots: int):
        super().__init__()

        self.node = node
        self.node_name_prefix = node.get_name()  # <-- use this as the prefix
        self.node.get_logger().info('cxrl_gym init')

        # Helper function to prepend the node name to topic/service names
        def prefixed(name: str) -> str:
            return f"{self.node_name_prefix}/{name}"

        # Service clients
        self.set_rl_mode_client = self.node.create_client(SetRLMode, prefixed('set_rl_mode'))
        self.end_training_client = self.node.create_client(EndTraining, prefixed('end_training'))
        self.get_action_list_executable_client = self.node.create_client(
            GetActionList, prefixed('get_action_list_executable')
        )
        self.get_action_list_executable_for_robot_client = self.node.create_client(
            GetActionListRobot, prefixed('get_action_list_executable_for_robot')
        )
        self.get_observable_objects_client = self.node.create_client(
            GetObservableObjects, prefixed('get_observable_objects')
        )
        self.get_observable_predicates_client = self.node.create_client(
            GetObservablePredicates, prefixed('get_observable_predicates')
        )
        self.get_predefined_observables_client = self.node.create_client(
            GetPredefinedObservables, prefixed('get_predefined_observables')
        )
        self.get_predefined_actions_client = self.node.create_client(
            GetPredefinedActions, prefixed('get_predefined_actions')
        )
        self.get_observable_actions_client = self.node.create_client(
            GetObservableActions, prefixed('get_observable_actions')
        )
        self.get_episode_end_client = self.node.create_client(
            GetEpisodeEnd, prefixed('get_episode_end')
        )
        self.get_env_state_client = self.node.create_client(GetEnvState, prefixed('get_env_state'))
        self.get_action_list_client = self.node.create_client(
            GetActionList, prefixed('get_action_list')
        )

        # Action clients
        self.reset_env_client = ActionClient(self.node, ResetEnv, prefixed('reset_env'))
        self.get_free_robot_client = ActionClient(
            self.node, GetFreeRobot, prefixed('get_free_robot')
        )
        self.action_selection_client = ActionClient(
            self.node, ActionSelection, prefixed('action_selection')
        )

        # initialize other internal state
        self.reset_env_result = None
        self.get_free_robot_result = None
        self.action_selection_send_goal_futures = [None] * number_robots
        self.action_selection_get_result_futures = [None] * number_robots
        self.action_selection_results = [None] * number_robots
        self.action_selection_goal_handles = [None] * number_robots

        self.time_sleep = 0.001
        self.shutdown = False
        self.number_of_robots = number_robots
        self.mode = mode
        self.rl_model = None
        self.next_robot = None
        self.robot_locked = False
        self.executable_actions_dicts_for_robot = {}
        self.executable_actions_dict = {}
        self.reset_wait_time = 3

        # Observation space
        obs_space = self.generate_observation_space()
        sorted_obs = sorted(set(obs_space))
        self.obs_dict = dict(zip(range(len(sorted_obs)), sorted_obs))
        self.inv_obs_dict = dict(zip(sorted_obs, range(len(sorted_obs))))
        self.n_obs = len(sorted_obs)
        self.observation_space = Box(0, 1, (self.n_obs,))

        # Action space
        action_space = self.generate_action_space()
        sorted_actions = ['no-op'] + sorted(set(action_space))
        self.action_dict = dict(zip(range(len(sorted_actions)), sorted_actions))
        self.inv_action_dict = dict(zip(sorted_actions, range(len(sorted_actions))))
        self.n_actions = len(sorted_actions)
        self.action_space = Discrete(self.n_actions)

        self.node.get_logger().info('cxrl_gym init complete')

    def on_training_end(self):
        while not self.end_training_client.wait_for_service(1.0):
            self.node.get_logger().info('Waiting for service (end_client) to be ready...')

        request = EndTraining.Request()
        future = self.end_training_client.call_async(request)

        while not future.done():
            time.sleep(self.time_sleep)

        future.result()

    """
            ===GYM-FUNCTIONS===
    """

    def step(self, action: int) -> tuple[npt.NDArray[np.float32], int, bool, bool, dict]:
        """
        Execute one environment step.

        This sends an action to the CX system and returns the resulting
        observation, reward, and done state.

        Parameters
        ----------
        action : int
            Index of the chosen action.

        Returns
        -------
        tuple
            observation : np.ndarray
                The next state observation.
            reward : float
                Scalar reward.
            terminated : bool
                Whether the episode ended normally.
            truncated : bool
                Whether the episode ended early.
            info : dict
                Additional debug information.

        """
        action_string = self.action_dict[action]

        self.node.get_logger().info(f'In step function with action {action}: {action_string}')

        if action_string in self.executable_actions_dicts_for_robot[self.next_robot]:
            action_id = self.executable_actions_dicts_for_robot[self.next_robot][action_string]
        else:
            self.node.get_logger().info(
                f'Action {action_string} not executable for robot \
                {self.next_robot}!'
            )
            self.robot_locked = False
            # state = self.get_env_state()
            state = self.get_observation()

            terminated, reward = self.get_episode_end()
            truncated = False
            info = {'outcome': 'NO-ACTION-FOR-ROBOT'}
            return state, reward, terminated, truncated, info

        action_msg = ActionSelection.Goal()
        action_msg.actionid = action_id
        robot_index = int(self.next_robot[5]) - 1
        self.action_selection_client.wait_for_server()
        self.action_selection_send_goal_futures[robot_index] = (
            self.action_selection_client.send_goal_async(
                action_msg,
                partial(self.action_selection_feedback_callback, robot_index=robot_index),
            )
        )
        self.action_selection_send_goal_futures[robot_index].add_done_callback(
            partial(self.action_selection_goal_response_callback, robot_index=robot_index)
        )

        while self.action_selection_results[robot_index] is None:
            if self.shutdown:
                self.node.get_logger().info('Shutdown triggered!')
                state = None
                reward = None
                terminated = None
                truncated = None
                info = {'outcome': 'SHUTDOWN'}
                return state, reward, terminated, truncated, info

            time.sleep(self.time_sleep)

        result = self.action_selection_results[robot_index]
        self.action_selection_results[robot_index] = None

        result_action_id = result.actionid
        result_reward = result.reward
        result_info = result.info

        self.node.get_logger().info('Getting observation of current state')
        state = self.get_observation()
        self.node.get_logger().info('Current state observed')

        self.node.get_logger().info(
            f'Action {result_action_id} finished with reward {result_reward}. {result_info}'
        )

        done = False
        truncated = False
        info = {}
        reward = result_reward
        if result_info == 'Done':
            info['outcome'] = 'Game Over'
            done = True
        elif result_info == 'Aborted':
            info['outcome'] = 'RESET'
        else:
            info['outcome'] = ''

        return state, reward, done, truncated, info

    def generate_observation_space(self) -> list[str]:
        self.node.get_logger().info('Generating observation space...')
        obs_space = []
        predefined_observables = self.get_predefined_observables()
        obs_space += predefined_observables
        observable_predicates_dict = self.get_observable_predicates()
        for predicate in observable_predicates_dict:
            observable_objects_for_param_name = {}
            types_for_param_name = observable_predicates_dict[predicate]

            if not types_for_param_name:
                continue

            observable_object_not_found = False
            for param_name in types_for_param_name:
                observable_objects_for_param_type = self.get_observable_objects(
                    types_for_param_name[param_name]
                )

                if observable_objects_for_param_type[0] == 'Not found':
                    observable_object_not_found = True

                observable_objects_for_param_name[param_name] = observable_objects_for_param_type

            if observable_object_not_found:
                continue
            obs_df = self.expand_grid(observable_objects_for_param_name)

            obs_df.insert(0, '(', '(')
            obs_df.insert(len(obs_df.columns), ')', ')')
            obs_df.insert(0, 'Predicate', predicate)

            obs_str = obs_df.to_string(header=False, index=False, index_names=False).split('\n')
            values = ['#'.join(element.split()) for element in obs_str]
            values = [w.replace('#(#', '(') for w in values]
            values = [w.replace('#)', ')') for w in values]

            obs_space += values

        self.node.get_logger().info(f'Observation space: {obs_space}')
        self.node.get_logger().info('Observation space size: ' + str(len(obs_space)))
        return obs_space

    def reset(
        self, seed: int = None, options: dict[str, any] = None
    ) -> tuple[npt.NDArray[np.float32], dict]:
        """
        Reset the environment and return the initial observation.

        Calls the `/reset_env` action and queries `/get_env_state`
        for the initial environment state.

        Parameters
        ----------
        seed : int, optional
            Random seed for reproducibility.
        options : dict, optional
            Additional reset parameters.

        Returns
        -------
        tuple
            observation : np.ndarray
                The initial observation.
            info : dict
                Metadata about the reset process.

        """
        self.node.get_logger().info('Resetting environment...')
        super().reset(seed=seed)
        result = self.reset_env()
        self.node.get_logger().info(result)

        state = self.get_observation()

        self.executable_actions_dicts_for_robot = {}
        self.executable_actions_dict = {}

        info = {}
        return (state, info)

    def close(self) -> None:
        """Clean up environment resources."""
        super().close()

    def render(self):
        """
        Render the environment (no-op).

        This environment is symbolic and does not provide visualization.
        """
        raise NotImplementedError()

    def action_masks(self) -> npt.NDArray[np.int8]:
        self.node.get_logger().info('Creating action masks...')
        while self.robot_locked:
            if self.shutdown:
                return np.zeros((self.n_actions), dtype=np.int8)
            time.sleep(self.time_sleep)
        self.robot_locked = True
        self.next_robot = self.get_free_robot()
        if self.next_robot == 'Aborted':
            self.node.get_logger().info('get_free_robot aborted, unlocking robot selection...')
            self.robot_locked = False
            return np.zeros((self.n_actions), dtype=np.int8)
        self.executable_actions_dicts_for_robot[self.next_robot] = (
            self.get_action_list_executable_for_robot(self.next_robot)
        )

        available_actions = np.zeros((self.n_actions), dtype=np.int8)
        for action in self.executable_actions_dicts_for_robot[self.next_robot].keys():
            pos = self.inv_action_dict.get(action)
            if pos is not None:
                available_actions[pos] = 1
        return available_actions

    """
            ===CLIENT-FUNCTIONS===
    """

    def set_rl_model(self, model) -> None:
        """
        Attach an RL model for autonomous action selection.

        Registers a service `/exec_action_selection` to let ROS query
        the model when actions are needed.

        Parameters
        ----------
        model : object
            Trained RL model supporting ``predict(observation)``.

        """
        self.rl_model = model
        self.exec_action_selection_service = self.node.create_service(
            ExecActionSelection,
            self.node_name_prefix + '/exec_action_selection',
            self.exec_action_selection,
        )
        self.set_rl_mode(self.mode)

    def set_rl_mode(self, mode: str) -> str:
        """
        Set the current reinforcement learning (RL) mode on the remote system.

        This method calls the `SetRLMode` ROS 2 service and waits until a confirmation
        is received. It blocks until the service is ready and the asynchronous call
        completes.

        Parameters
        ----------
        mode : str
            The RL mode to set (e.g., "train", "eval").

        Returns
        -------
        str
            Confirmation message returned by the service.

        """
        while not self.set_rl_mode_client.wait_for_service(1.0):
            self.node.get_logger().info('Waiting for service (set_rl_mode) to be ready...')

        request = SetRLMode.Request()
        request.mode = mode

        future = self.set_rl_mode_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()

        mode_confirm = response.confirmation
        return mode_confirm

    def get_action_list_executable(self) -> dict:
        """
        Retrieve the list of all executable actions.

        Calls the `GetActionList` service to obtain all available actions that can
        currently be executed by the system.

        Returns
        -------
            dict: A mapping of action names to their corresponding action IDs.

        """
        while not self.get_action_list_executable_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_action_list_executable) to be ready...'
            )

        request = GetActionList.Request()

        future = self.get_action_list_executable_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        dict_action_ids = self.unpack_transmitted_action_list(response.actions)
        return dict_action_ids

    def get_action_list_executable_for_robot(self, robot: str) -> dict:
        """
        Retrieve the list of executable actions for a specific robot.

        Calls the `GetActionListRobot` service for the given robot and waits for
        the result.

        Parameters
        ----------
        robot : str
            The robot identifier for which to retrieve executable actions.

        Returns
        -------
        dict
            A mapping of action names to action IDs for the specified robot.

        """
        while not self.get_action_list_executable_for_robot_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_action_list_executable_for_robot) to be ready...'
            )

        request = GetActionListRobot.Request()
        request.robot = robot

        future = self.get_action_list_executable_for_robot_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        dict_action_ids = self.unpack_transmitted_action_list(response.actions)
        return dict_action_ids

    def get_observable_objects(self, obj_type: str) -> list[str]:
        """
        Retrieve all observable objects of a given type.

        Calls the `GetObservableObjects` service to get a list of currently observable
        objects matching the specified type.

        Parameters
        ----------
        obj_type : str
            The type of observable object (e.g., "robot", "item").

        Returns
        -------
        list[str]
            List of observable object names.

        """
        while not self.get_observable_objects_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_observable_objects) to be ready...'
            )

        request = GetObservableObjects.Request()
        request.type = obj_type

        future = self.get_observable_objects_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()

        observable_objects = response.objects
        return observable_objects

    def get_observable_predicates(self) -> dict:
        """
        Retrieve all observable predicates and their parameters.

        Calls the `GetObservablePredicates` service and constructs a dictionary mapping
        predicate names to their parameter information.

        Returns
        -------
            dict: Dictionary where keys are predicate names and values describe parameter
                  counts, names, and types.

        """
        while not self.get_observable_predicates_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_observable_predicates) to be ready...'
            )

        request = GetObservablePredicates.Request()

        future = self.get_observable_predicates_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        observable_predicates = self.create_observable_dict(
            response.predicate_names,
            response.param_counts,
            response.param_names,
            response.param_types,
        )
        return observable_predicates

    def get_predefined_observables(self) -> list:
        """
        Retrieve the list of predefined observables.

        Calls the `GetPredefinedObservables` service to get predefined observables used
        for RL environment construction or monitoring.

        Returns
        -------
            list: A list of predefined observables.

        """
        while not self.get_predefined_observables_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_predefined_observables) to be ready...'
            )

        request = GetPredefinedObservables.Request()

        future = self.get_predefined_observables_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        return response.observables

    def get_predefined_actions(self) -> list:
        """
        Retrieve the list of predefined observables.

        Calls the `GetPredefinedActions` service to get predefined observables used
        for RL environment construction or monitoring.

        Returns
        -------
            list: A list of predefined actions.

        """
        while not self.get_predefined_actions_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_predefined_actions) to be ready...'
            )

        request = GetPredefinedActions.Request()

        future = self.get_predefined_actions_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        return response.actions

    def get_observable_actions(self) -> dict:
        """
        Retrieve all observable actions and their parameters.

        Calls the `GetObservableActions` service and constructs a dictionary mapping
        predicate names to their parameter information.

        Returns
        -------
            dict: Dictionary where keys are predicate names and values describe parameter
                  counts, names, and types.

        """
        while not self.get_observable_actions_client.wait_for_service(1.0):
            self.node.get_logger().info(
                'Waiting for service (get_observable_actions) to be ready...'
            )

        request = GetObservableActions.Request()

        future = self.get_observable_actions_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()
        observable_actions = self.create_observable_dict(
            response.action_names,
            response.param_counts,
            response.param_names,
            response.param_types,
        )
        return observable_actions

    def generate_action_space(self) -> list[str]:
        """
        Generate the RL action space dynamically.

        This includes predefined actions as well as actions with parameters,
        which are expanded over all currently observable objects of the corresponding types.

        Returns
        -------
        list[str]
            A list of action strings for the RL environment.
        """
        self.node.get_logger().info('Generating action space...')
        action_space = []

        # Step 1: Add predefined actions
        predefined_actions = (
            self.get_predefined_actions()
        )  # assumes a service call similar to get_predefined_observables
        action_space += predefined_actions

        # Step 2: Add parameterized actions
        observable_actions_dict = (
            self.get_observable_actions()
        )  # returns dict: {action_name: {param_name: type}}
        for action_name, param_types in observable_actions_dict.items():
            if not param_types:
                continue

            # Retrieve observable objects for each parameter type
            observable_objects_for_param_name = {}
            skip_action = False
            for param_name, obj_type in param_types.items():
                objects = self.get_observable_objects(obj_type)
                if not objects or objects[0] == 'Not found':
                    skip_action = True
                    break
                observable_objects_for_param_name[param_name] = objects

            if skip_action:
                continue

            # Compute Cartesian product of all parameter values
            action_df = self.expand_grid(observable_objects_for_param_name)

            # Add action name and parentheses for consistency
            action_df.insert(0, '(', '(')
            action_df.insert(len(action_df.columns), ')', ')')
            action_df.insert(0, 'Action', action_name)

            # Convert each row to a single action string
            action_strings = action_df.to_string(
                header=False, index=False, index_names=False
            ).split('\n')
            action_strings = ['#'.join(element.split()) for element in action_strings]
            action_strings = [w.replace('#(#', '(').replace('#)', ')') for w in action_strings]

            action_space += action_strings

        self.node.get_logger().info(f'Action space: {action_space}')
        self.node.get_logger().info('Action space size: ' + str(len(action_space)))
        return action_space

    def get_env_state(self) -> str:
        """
        Create a new RL environment state.

        Calls the `GetEnvState` service to generate a new environment state for
        reinforcement learning execution.

        Returns
        -------
            str: The generated environment state as a serialized string.

        """
        while not self.get_env_state_client.wait_for_service(1.0):
            self.node.get_logger().info('Waiting for service (get_env_state) to be ready...')

        request = GetEnvState.Request()
        future = self.get_env_state_client.call_async(request)
        while not future.done():
            time.sleep(self.time_sleep)
        response = future.result()

        rl_env_state = response.state
        return rl_env_state

    def get_episode_end(self) -> tuple[bool, int]:
        """
        Check whether the current RL episode has ended.

        Calls the `CheckForEpisodeEnd` service to determine if the episode
        should terminate and obtain the corresponding reward.

        Returns
        -------
            Tuple[bool, int]: (episode_end, reward)
                - episode_end (bool): Whether the episode has ended.
                - reward (int): Reward for the transition.

        """
        while not self.get_episode_end_client.wait_for_service(1.0):
            self.node.get_logger().info('Waiting for service (get_episode_end) to be ready...')

        request = GetEpisodeEnd.Request()
        future = self.get_episode_end_client.call_async(request)

        while not future.done():
            time.sleep(self.time_sleep)

        response = future.result()

        return response.episode_end, response.reward

    def exec_action_selection(
        self, request: ExecActionSelection.Request, response: ExecActionSelection.Response
    ) -> ExecActionSelection.Response:
        """
        Execute the RL-based action selection process.

        This method receives a serialized environment state, converts it into an
        observation for the RL model, predicts the next action using the model, and
        fills the ROS 2 service response with the selected action ID.

        Parameters
        ----------
        request : ExecActionSelection.Request
            The incoming request containing the serialized environment state and a list
            of executable actions.
        response : ExecActionSelection.Response
            The response object to populate.

        Returns
        -------
        ExecActionSelection.Response
            The populated response containing the selected action ID.

        """
        self.node.get_logger().info('Selecting action...')

        raw_facts = ast.literal_eval(request.state)
        observation = self.get_state_from_facts(raw_facts)
        self.executable_actions_dict = self.unpack_transmitted_action_list(request.actions)
        action_mask = np.zeros((self.n_actions), dtype=int)
        for action in self.executable_actions_dict.keys():
            pos = self.inv_action_dict.get(action)
            if pos is not None:
                action_mask[pos] = 1

        if np.sum(action_mask) == 0:
            action_mask[0] = 1
            response.actionid = 'no-op'
            return response

        action, _ = self.rl_model.predict(
            observation,
            deterministic=True,
            action_masks=action_mask,
        )

        action_string = self.action_dict[int(action)]
        response.actionid = self.executable_actions_dict[action_string]
        return response

    def reset_env(self) -> str:
        """
        Reset the CLIPS executive (CX) node.

        Sends a goal to the `ResetEnv` action server and waits for completion.

        Returns
        -------
            str: Confirmation message from the CX node after reset.

        """
        goal_msg = ResetEnv.Goal()
        self.reset_env_client.wait_for_server()
        self.reset_env_send_goal_future = self.reset_env_client.send_goal_async(
            goal_msg, self.reset_env_feedback_callback
        )
        self.reset_env_send_goal_future.add_done_callback(self.reset_env_goal_response_callback)
        while self.reset_env_result is None:
            time.sleep(self.time_sleep)
        reset_confirm = self.reset_env_result.confirmation
        return reset_confirm

    def reset_env_goal_response_callback(self, future: Future) -> None:
        """
        Handle the goal response from the `ResetEnv` action server via callback.

        Parameters
        ----------
        future : Future
            The future object representing the goal response.

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('reset_env rejected')
            return
        self.node.get_logger().info('reset_env accepted')
        self.reset_env_goal_handle = goal_handle
        self.reset_env_get_result_future = goal_handle.get_result_async()
        self.reset_env_get_result_future.add_done_callback(self.reset_env_get_result_callback)

    def reset_env_get_result_callback(self, future: Future) -> None:
        """
        Handle the result returned from the `ResetEnv` action via callback.

        Parameters
        ----------
        future : Future
                The future containing the action result.

        """
        self.node.get_logger().info('Result for reset_env received')
        self.reset_env_result = future.result().result

    def reset_env_feedback_callback(self, feedback_msg: ResetEnv.Feedback) -> None:
        """
        Handle feedback messages from the `ResetEnv` action via callback.

        Parameters
        ----------
        feedback_msg : ResetEnv.Feedback
                The feedback message received from the action.

        """
        feedback = feedback_msg.feedback.feedback
        self.node.get_logger().info(feedback)

    def reset_env_cancel_done(self, future: Future) -> None:
        """
        Invoke once a cancel request for `ResetEnv` is completed via callback.

        Parameters
        ----------
        future : Future
                The future representing the cancel result.

        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('reset_env canceled')
        else:
            self.node.get_logger().info('Failed to cancel reset_env!')

    def get_free_robot(self) -> str:
        """
        Retrieve a currently available robot.

        Sends a goal to the `GetFreeRobot` action server and waits for the result.

        Returns
        -------
        str
                The name or identifier of the free robot.

        """
        goal_msg = GetFreeRobot.Goal()
        self.get_free_robot_client.wait_for_server()
        self.get_free_robot_send_goal_future = self.get_free_robot_client.send_goal_async(
            goal_msg, self.get_free_robot_feedback_callback
        )
        self.get_free_robot_send_goal_future.add_done_callback(
            self.get_free_robot_goal_response_callback
        )
        while self.get_free_robot_result is None:
            time.sleep(self.time_sleep)
        robot = self.get_free_robot_result.robot
        self.get_free_robot_result = None
        return robot

    def get_free_robot_goal_response_callback(self, future: Future) -> None:
        """
        Handle goal responses from the `GetFreeRobot` action server via callback.

        Parameters
        ----------
        future : Future
                The future representing the goal response.

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('get_free_robot rejected')
            return
        self.node.get_logger().info('get_free robot accepted')
        self.get_free_robot_goal_handle = goal_handle
        self.get_free_robot_get_result_future = goal_handle.get_result_async()
        self.get_free_robot_get_result_future.add_done_callback(
            self.get_free_robot_get_result_callback
        )

    def get_free_robot_get_result_callback(self, future: Future) -> None:
        """
        Handle the result of the `GetFreeRobot` action via callback.

        Parameters
        ----------
        future : Future
                The future containing the action result.

        """
        self.node.get_logger().info('Result for get_free_robot received')
        self.get_free_robot_result = future.result().result

    def get_free_robot_feedback_callback(self, feedback_msg: GetFreeRobot.Feedback) -> None:
        """
        Handle feedback messages from the `GetFreeRobot` action via callback.

        Parameters
        ----------
        feedback_msg : GetFreeRobot.Feedback
                The feedback message received from the action.

        """
        feedback = feedback_msg.feedback.feedback
        self.node.get_logger().info(feedback)

    def get_free_robot_cancel_done(self, future: Future) -> None:
        """
        Invoke via callback once a cancel request for `GetFreeRobot` is completed.

        Parameters
        ----------
        future : Future
                The future representing the cancel result.

        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('get_free_robot canceled')
        else:
            self.node.get_logger().info('Failed to cancel get_free_robot!')

    def action_selection_goal_response_callback(self, future: Future, robot_index: int) -> None:
        """
        Handle goal responses from the action selection server via callback.

        Parameters
        ----------
        future : Future
                The future representing the goal response.
        robot_index : int
                Index of the robot associated with this goal.

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Action selection rejected')
            return
        self.node.get_logger().info('Action selection accepted')
        self.action_selection_goal_handles[robot_index] = goal_handle
        self.action_selection_get_result_futures[robot_index] = goal_handle.get_result_async()
        self.action_selection_get_result_futures[robot_index].add_done_callback(
            partial(self.action_selection_get_result_callback, robot_index=robot_index)
        )

    def action_selection_get_result_callback(self, future: Future, robot_index: int) -> None:
        """
        Handle action selection results via callback.

        Parameters
        ----------
        future : Future
                The future containing the action result.
        robot_index : int
                Index of the robot for which the result applies.

        """
        self.action_selection_results[robot_index] = future.result().result
        self.node.get_logger().info(
            f'Result for action {self.action_selection_results[robot_index].actionid} received'
        )

    def action_selection_feedback_callback(
        self, feedback_msg: ActionSelection.Feedback, robot_index: int
    ) -> None:
        """
        Handle feedback messages during action selection via callback.

        If the feedback indicates that an action selection fact was asserted,
        the robot is unlocked for further use.

        Parameters
        ----------
        feedback_msg : ActionSelection.Feedback
                Feedback message from the action.
        robot_index : int
                Index of the robot for which feedback applies.

        """
        feedback = feedback_msg.feedback.feedback
        if feedback == 'Action selection fact asserted':
            self.robot_locked = False
            self.node.get_logger().info(
                f'Action selection fact for robot{robot_index+1} asserted, \
                unlocking robot selection...'
            )
            return
        self.node.get_logger().info(feedback)

    def action_selection_cancel_done(self, future: Future, robot_index: int) -> None:
        """
        Invoke via callback once a cancel request for action selection is completed.

        Parameters
        ----------
        future : Future
                The future representing the cancel result.
        robot_index : int
                Index of the robot for which the cancel applies.

        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f'Action selection for robot{robot_index+1} canceled')
        else:
            self.node.get_logger().info(
                f'Failed to cancel action selection for robot{robot_index+1}!'
            )

    """
            ===HELPER-FUNCTIONS===
    """

    def get_observation(self) -> npt.NDArray[np.float32]:
        """
        Generate the current RL observation vector.

        Creates a new RL environment state using `get_env_state()`,
        parses the returned fact string into Python objects, and converts
        them into a numerical observation vector suitable for the RL model.

        Returns
        -------
        numpy.ndarray
                The observation vector as a NumPy array of type float32.

        """
        fact_string = self.get_env_state()
        raw_facts = ast.literal_eval(fact_string)
        return self.get_state_from_facts(raw_facts)

    def expand_grid(self, dictionary: dict) -> pd.DataFrame:
        """
        Generate a Cartesian product (grid) of all dictionary value combinations.

        Each key in the input dictionary represents a column, and its list of values
        defines the possible entries.

        Parameters
        ----------
        dictionary : dict
                A dictionary where each key maps to a list of possible values.

        Returns
        -------
        pandas.DataFrame
                A DataFrame containing one row per value combination.

        """
        return pd.DataFrame(list(product(*dictionary.values())), columns=dictionary.keys())

    def create_observable_dict(
        self,
        names: list[str],
        param_counts: list[int],
        param_names: list[str],
        param_types: list[str],
    ) -> dict:
        """
        Construct a dictionary describing entities and their parameters.

        This method takes parallel lists describing either observable predicates
        (for observation space) or parameterized actions (for action space),
        and aggregates them into a nested dictionary for easier lookup.

        Parameters
        ----------
        names : list[str]
            Names of the predicates or actions.
        param_counts : list[int]
            Number of parameters for each predicate or action.
        param_names : list[str]
            Flat list of parameter names for all predicates or actions.
        param_types : list[str]
            Flat list of parameter types corresponding to each name.

        Returns
        -------
        dict
            A mapping from entity names (predicates or actions) to dictionaries
            of parameter names and types.
        """
        entities = {}
        param_counter = 0
        for x in range(len(names)):
            name = names[x]
            number_params = param_counts[x]
            params = {}
            param_range = param_counter + number_params
            while param_counter < param_range:
                params[param_names[param_counter]] = param_types[param_counter]
                param_counter += 1
            entities[name] = params
        return entities

    def unpack_transmitted_action_list(self, action_list: list[str]) -> dict:
        """
        Parse a list of encoded action strings into a dictionary mapping names to IDs.

        Each action string is expected to have the format:
        ``"<action_id>|<action_name>"``.

        Parameters
        ----------
        action_list : list[str]
                List of encoded action strings.

        Returns
        -------
        dict
                Mapping from action names to action IDs.

        """
        dict_action_ids = {}
        for action in action_list:
            action_splitted = action.split('|')

            action_id = action_splitted[0]
            action_name = action_splitted[1]

            dict_action_ids[action_name] = action_id

        return dict_action_ids

    def get_state_from_facts(self, obs_facts) -> npt.NDArray[np.float32]:
        """
        Convert a set of observation facts into a binary state vector.

        Each fact is matched against the internal observation dictionary.
        The corresponding index is set to 1.0 if the fact is present, 0.0 otherwise.

        Parameters
        ----------
        obs_facts : iterable
                Collection of fact strings representing the current state.

        Returns
        -------
        numpy.ndarray
                Binary state vector as a NumPy array of type float32.

        """
        new_state = np.zeros(self.n_obs, dtype=np.float32)
        for fact in obs_facts:
            if self.inv_obs_dict.get(fact) is not None:
                pos = self.inv_obs_dict[fact]
                new_state[pos] = 1.0

        return new_state
