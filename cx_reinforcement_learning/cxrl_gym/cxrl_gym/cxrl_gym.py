from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from cx_rl_interfaces.srv import GetActionList, GetObservableObjects, GetObservablePredicates, GetPredefinedObservables, GetActionListRobot, CreateRLEnvState, SetRLMode, ExecActionSelection
from cx_rl_interfaces.action import ActionSelection, GetFreeRobot, ResetCX

from gymnasium import Env
from gymnasium.spaces import Discrete, Box

from functools import partial
from itertools import product
import ast
import numpy as np
import numpy.typing as npt
import time
import pandas as pd


class CXRLGym(Env):
    def __init__(self, node: Node, mode: str, number_robots: int):
        super().__init__()

        self.node = node
        self.node.get_logger().info("cxrl_gym init")

        self.set_rl_mode_client = self.node.create_client(
            SetRLMode, '/set_rl_mode')
        self.get_action_list_executable_client = self.node.create_client(
            GetActionList, '/get_action_list_executable')
        self.get_action_list_executable_for_robot_client = self.node.create_client(
            GetActionListRobot, '/get_action_list_executable_for_robot')
        self.get_observable_objects_client = self.node.create_client(
            GetObservableObjects, '/get_observable_objects')
        self.get_observable_predicates_client = self.node.create_client(
            GetObservablePredicates, '/get_observable_predicates')
        self.get_predefined_observables_client = self.node.create_client(
            GetPredefinedObservables, '/get_predefined_observables')
        self.create_rl_env_state_client = self.node.create_client(
            CreateRLEnvState, '/create_rl_env_state')

        self.reset_cx_client = ActionClient(self.node, ResetCX, "/reset_cx")
        self.reset_cx_result = None
        self.reset_cx_send_goal_future = None
        self.reset_cx_get_result_future = None
        self.reset_cx_goal_handle = None

        self.get_free_robot_client = ActionClient(
            self.node, GetFreeRobot, "/get_free_robot")
        self.get_free_robot_result = None
        self.get_free_robot_send_goal_future = None
        self.get_free_robot_get_result_future = None
        self.get_free_robot_goal_handle = None

        self.time_sleep = 0.001
        self.shutdown = False

        self.number_of_robots = number_robots
        self.mode = mode

        self.rl_model = None

        self.action_selection_send_goal_futures = []
        self.action_selection_get_result_futures = []
        self.action_selection_results = []
        self.action_selection_goal_handles = []

        self.action_selection_client = ActionClient(
            self.node, ActionSelection, f"/action_selection")
        for i in range(self.number_of_robots):
            self.action_selection_send_goal_futures.append(None)
            self.action_selection_get_result_futures.append(None)
            self.action_selection_results.append(None)
            self.action_selection_goal_handles.append(None)

        self.next_robot = "None"
        self.robot_locked = False
        self.executable_actions_dicts_for_robot = {}
        self.executable_actions_dict = {}
        self.reset_wait_time = 3

        # Observation space
        obs_space = self.generate_observation_space()
        sorted_obs = sorted(set(obs_space))
        set_keys_obs = range(0, len(sorted_obs))
        self.obs_dict = dict(zip(set_keys_obs, sorted_obs))
        self.inv_obs_dict = (dict(zip(sorted_obs, set_keys_obs)))
        self.n_obs = len(sorted_obs)
        self.observation_space = Box(0, 1, (self.n_obs,))

        # Action space
        action_space = self.generate_action_space()
        sorted_actions = sorted(set(action_space))
        set_keys_action = range(0, len(sorted_actions))
        self.action_dict = dict(zip(set_keys_action, sorted_actions))
        self.inv_action_dict = dict(zip(sorted_actions, set_keys_action))
        self.n_actions = len(sorted_actions)
        self.action_space = Discrete(self.n_actions)

        self.node.get_logger().info("cxrl_gym init complete")

    """
            ===GYM-FUNCTIONS===
    """

    def step(self, action: int) -> tuple[npt.NDArray[np.float32], int, bool, bool, dict]:

        action_string = self.action_dict[action]

        self.node.get_logger().info(
            f"In step function with action {action}: {action_string}")

        if action_string in self.executable_actions_dicts_for_robot[self.next_robot]:
            action_id = self.executable_actions_dicts_for_robot[self.next_robot][action_string]
        else:
            self.node.get_logger().info(
                f"Action {action_string} not executable for robot {self.next_robot}!")
            self.robot_locked = False
            state = self.create_rl_env_state()
            reward = 0
            terminated = False
            truncated = False
            info = {"outcome": "NO-GOAL-ID"}
            return state, reward, terminated, truncated, info

        action_msg = ActionSelection.Goal()
        action_msg.actionid = action_id
        robot_index = int(self.next_robot[5]) - 1
        self.action_selection_client.wait_for_server()
        self.action_selection_send_goal_futures[robot_index] = self.action_selection_client.send_goal_async(
            action_msg, partial(self.action_selection_feedback_callback, robot_index=robot_index))
        self.action_selection_send_goal_futures[robot_index].add_done_callback(
            partial(self.action_selection_goal_response_callback, robot_index=robot_index))

        while self.action_selection_results[robot_index] == None:
            if self.shutdown:
                self.node.get_logger().info(f"Shutdown triggered!")
                state = None
                reward = None
                terminated = None
                truncated = None
                info = {"outcome": "SHUTDOWN"}
                return state, reward, terminated, truncated, info

            time.sleep(self.time_sleep)

        result = self.action_selection_results[robot_index]
        self.action_selection_results[robot_index] = None

        result_action_id = result.actionid
        result_reward = result.reward
        result_info = result.info

        self.node.get_logger().info(f"Getting observation of current state")
        state = self.get_observation()
        self.node.get_logger().info(f"Current state observed")

        self.node.get_logger().info(
            f"Action {result_action_id} finished with reward {result_reward}. {result_info}")

        done = False
        truncated = False
        info = {}
        reward = result_reward
        if result_info == "Done":
            info["outcome"] = "Game Over"
            done = True
        elif result_info == "Aborted":
            info["outcome"] = "RESET"
        else:
            info["outcome"] = ""

        return state, reward, done, truncated, info

    def generate_action_space(self) -> list[str]:
        raise NotImplementedError()

    def generate_observation_space(self) -> list[str]:
        self.node.get_logger().info("Generating observation space...")
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
                    types_for_param_name[param_name])

                if observable_objects_for_param_type[0] == "Not found":
                    observable_object_not_found = True

                observable_objects_for_param_name[param_name] = observable_objects_for_param_type

            if observable_object_not_found:
                continue
            obs_df = self.expand_grid(observable_objects_for_param_name)

            obs_df.insert(0, '(', '(')
            obs_df.insert(len(obs_df.columns), ')', ')')
            obs_df.insert(0, 'Predicate', predicate)

            obs_str = obs_df.to_string(
                header=False, index=False, index_names=False).split('\n')
            values = ['#'.join(element.split()) for element in obs_str]
            values = [w.replace('#(#', '(') for w in values]
            values = [w.replace('#)', ')') for w in values]

            obs_space += values

        self.node.get_logger().info(f"Observation space: {obs_space}")
        self.node.get_logger().info("Observation space size: "+str(len(obs_space)))
        return obs_space

    def reset(self, seed: int = None, options: dict[str, any] = None) -> tuple[npt.NDArray[np.float32], dict]:
        self.node.get_logger().info("Resetting environment...")
        super().reset(seed=seed)
        result = self.reset_cx()
        self.node.get_logger().info(result)

        state = self.get_observation()

        self.executable_actions_dicts_for_robot = {}
        self.executable_actions_dict = {}

        info = {}
        return (state, info)

    def close(self) -> None:
        super().close()

    def render(self):
        raise NotImplementedError()

    def action_masks(self) -> npt.NDArray[np.int8]:
        self.node.get_logger().info("Creating action masks...")
        while self.robot_locked:
            if self.shutdown:
                return np.zeros((self.n_actions), dtype=np.int8)
            time.sleep(self.time_sleep)
        self.robot_locked = True
        self.next_robot = self.get_free_robot()
        if self.next_robot == "Aborted":
            self.node.get_logger().info(
                f"get_free_robot aborted, unlocking robot selection...")
            self.robot_locked = False
            return np.zeros((self.n_actions), dtype=np.int8)
        self.executable_actions_dicts_for_robot[self.next_robot] = self.get_action_list_executable_for_robot(
            self.next_robot)

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
        self.rl_model = model
        self.exec_action_selection_service = self.node.create_service(
            ExecActionSelection, '/exec_action_selection', self.exec_action_selection)
        self.set_rl_mode(self.mode)

    def set_rl_mode(self, mode: str) -> str:
        while not self.set_rl_mode_client.wait_for_service(1.0):
            self.node.get_logger().info("Waiting for service (set_rl_mode) to be ready...")

        request = SetRLMode.Request()
        request.mode = mode

        future = self.set_rl_mode_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()

        mode_confirm = response.confirmation
        return mode_confirm

    def get_action_list_executable(self) -> dict:
        while not self.get_action_list_executable_client.wait_for_service(1.0):
            self.node.get_logger().info(
                "Waiting for service (get_action_list_executable) to be ready...")

        request = GetActionList.Request()

        future = self.get_action_list_executable_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()
        dict_action_ids = self.unpack_transmitted_action_list(response.actions)
        return dict_action_ids

    def get_action_list_executable_for_robot(self, robot: str) -> dict:
        while not self.get_action_list_executable_for_robot_client.wait_for_service(1.0):
            self.node.get_logger().info(
                "Waiting for service (get_action_list_executable_for_robot) to be ready...")

        request = GetActionListRobot.Request()
        request.robot = robot

        future = self.get_action_list_executable_for_robot_client.call_async(
            request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()
        dict_action_ids = self.unpack_transmitted_action_list(response.actions)
        return dict_action_ids

    def get_observable_objects(self, type: str) -> list[str]:
        while not self.get_observable_objects_client.wait_for_service(1.0):
            self.node.get_logger().info("Waiting for service (get_observable_objects) to be ready...")

        request = GetObservableObjects.Request()
        request.type = type

        future = self.get_observable_objects_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()

        observable_objects = response.objects
        return observable_objects

    def get_observable_predicates(self) -> dict:
        while not self.get_observable_predicates_client.wait_for_service(1.0):
            self.node.get_logger().info(
                "Waiting for service (get_observable_predicates) to be ready...")

        request = GetObservablePredicates.Request()

        future = self.get_observable_predicates_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()
        observable_predicates = self.create_observable_predicate_dict(
            response.predicatenames, response.paramcounts, response.paramnames, response.paramtypes)
        return observable_predicates
    
    def get_predefined_observables(self) -> list:
        while not self.get_predefined_observables_client.wait_for_service(1.0):
            self.node.get_logger().info(
                "Waiting for service (get_predefined_observables) to be ready...")

        request = GetPredefinedObservables.Request()

        future = self.get_predefined_observables_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()
        return response.observables

    def create_rl_env_state(self) -> str:
        while not self.create_rl_env_state_client.wait_for_service(1.0):
            self.node.get_logger().info("Waiting for service (create_rl_env_state) to be ready...")

        request = CreateRLEnvState.Request()
        future = self.create_rl_env_state_client.call_async(request)
        while (not future.done()):
            time.sleep(self.time_sleep)
        response = future.result()

        rl_env_state = response.state
        return rl_env_state

    def exec_action_selection(self, request: ExecActionSelection.Request, response: ExecActionSelection.Response) -> ExecActionSelection.Response:
        self.node.get_logger().info("Selecting action...")

        raw_facts = ast.literal_eval(request.state)
        observation = self.get_state_from_facts(raw_facts)
        self.executable_actions_dict = self.unpack_transmitted_action_list(
            request.actions)
        action_mask = np.zeros((self.n_actions), dtype=int)
        for action in self.executable_actions_dict.keys():
            pos = self.inv_action_dict.get(action)
            if pos is not None:
                action_mask[pos] = 1

        action, _ = self.rl_model.predict(
            observation,
            deterministic=True,
            action_masks=action_mask,
        )

        action_string = self.action_dict[int(action)]
        response.actionid = self.executable_actions_dict[action_string]
        return response

    def reset_cx(self) -> str:
        goal_msg = ResetCX.Goal()
        self.reset_cx_client.wait_for_server()
        self.reset_cx_send_goal_future = self.reset_cx_client.send_goal_async(
            goal_msg, self.reset_cx_feedback_callback)
        self.reset_cx_send_goal_future.add_done_callback(
            self.reset_cx_goal_response_callback)
        while self.reset_cx_result == None:
            time.sleep(self.time_sleep)
        reset_confirm = self.reset_cx_result.confirmation
        return reset_confirm

    def reset_cx_goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("reset_cx rejected")
            return
        self.node.get_logger().info("reset_cx accepted")
        self.reset_cx_goal_handle = goal_handle
        self.reset_cx_get_result_future = goal_handle.get_result_async(
        )
        self.reset_cx_get_result_future.add_done_callback(
            self.reset_cx_get_result_callback)

    def reset_cx_get_result_callback(self, future: Future) -> None:
        self.node.get_logger().info(
            f"Result for reset_cx received")
        self.reset_cx_result = future.result().result

    def reset_cx_feedback_callback(self, feedback_msg: ResetCX.Feedback) -> None:
        feedback = feedback_msg.feedback.feedback
        self.node.get_logger().info(feedback)

    def reset_cx_cancel_done(self, future: Future) -> None:
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(
                f"reset_cx canceled")
        else:
            self.node.get_logger().info(
                f"Failed to cancel reset_cx!")

    def get_free_robot(self) -> str:
        goal_msg = GetFreeRobot.Goal()
        self.get_free_robot_client.wait_for_server()
        self.get_free_robot_send_goal_future = self.get_free_robot_client.send_goal_async(
            goal_msg, self.get_free_robot_feedback_callback)
        self.get_free_robot_send_goal_future.add_done_callback(
            self.get_free_robot_goal_response_callback)
        goal_canceled = False
        while self.get_free_robot_result == None:
            time.sleep(self.time_sleep)
        robot = self.get_free_robot_result.robot
        self.get_free_robot_result = None
        return robot

    def get_free_robot_goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("get_free_robot rejected")
            return
        self.node.get_logger().info("get_free robot accepted")
        self.get_free_robot_goal_handle = goal_handle
        self.get_free_robot_get_result_future = goal_handle.get_result_async(
        )
        self.get_free_robot_get_result_future.add_done_callback(
            self.get_free_robot_get_result_callback)

    def get_free_robot_get_result_callback(self, future: Future) -> None:
        self.node.get_logger().info(
            f"Result for get_free_robot received")
        self.get_free_robot_result = future.result().result

    def get_free_robot_feedback_callback(self, feedback_msg: GetFreeRobot.Feedback) -> None:
        feedback = feedback_msg.feedback.feedback
        self.node.get_logger().info(feedback)

    def get_free_robot_cancel_done(self, future: Future) -> None:
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(
                f"get_free_robot canceled")
        else:
            self.node.get_logger().info(
                f"Failed to cancel get_free_robot!")

    def action_selection_goal_response_callback(self, future: Future, robot_index: int) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Action selection rejected")
            return
        self.node.get_logger().info("Action selection accepted")
        self.action_selection_goal_handles[robot_index] = goal_handle
        self.action_selection_get_result_futures[robot_index] = goal_handle.get_result_async(
        )
        self.action_selection_get_result_futures[robot_index].add_done_callback(
            partial(self.action_selection_get_result_callback, robot_index=robot_index))

    def action_selection_get_result_callback(self, future: Future, robot_index: int) -> None:
        self.action_selection_results[robot_index] = future.result().result
        self.node.get_logger().info(
            f"Result for action {self.action_selection_results[robot_index].actionid} received")

    def action_selection_feedback_callback(self, feedback_msg: ActionSelection.Feedback, robot_index: int) -> None:
        feedback = feedback_msg.feedback.feedback
        if feedback == "Action selection fact asserted":
            self.robot_locked = False
            self.node.get_logger().info(
                f"Action selection fact for robot{robot_index+1} asserted, unlocking robot selection...")
            return
        self.node.get_logger().info(feedback)

    def action_selection_cancel_done(self, future: Future, robot_index: int) -> None:
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(
                f"Action selection for robot{robot_index+1} canceled")
        else:
            self.node.get_logger().info(
                f"Failed to cancel action selection for robot{robot_index+1}!")

    """
            ===HELPER-FUNCTIONS===
    """

    def get_observation(self) -> npt.NDArray[np.float32]:
        fact_string = self.create_rl_env_state()
        raw_facts = ast.literal_eval(fact_string)
        return self.get_state_from_facts(raw_facts)

    def expand_grid(self, dictionary: dict) -> pd.DataFrame:
        return pd.DataFrame([row for row in product(*dictionary.values())],
                            columns=dictionary.keys())

    def create_observable_predicate_dict(self,
                                     predicate_names: list[str],
                                     param_counts: list[int],
                                     param_names: list[str],
                                     param_types: list[str]) -> dict:
        predicates = {}
        param_counter = 0
        for x in range(len(predicate_names)):
            name = predicate_names[x]
            number_params = param_counts[x]
            params = {}
            param_range = param_counter + number_params
            while param_counter < param_range:
                params[param_names[param_counter]] = param_types[param_counter]
                param_counter += 1
            predicates[name] = params
        return predicates

    def unpack_transmitted_action_list(self, action_list: list[str]) -> dict:
        dict_action_ids = {}
        for action in action_list:
            action_splitted = action.split("|")

            action_id = action_splitted[0]
            action_name = action_splitted[1]

            dict_action_ids[action_name] = action_id

        return dict_action_ids

    def get_state_from_facts(self, obs_facts) -> npt.NDArray[np.float32]:
        new_state = np.zeros(self.n_obs, dtype=np.float32)
        for fact in obs_facts:
            if self.inv_obs_dict.get(fact) is not None:
                pos = self.inv_obs_dict[fact]
                new_state[pos] = 1.0

        return new_state