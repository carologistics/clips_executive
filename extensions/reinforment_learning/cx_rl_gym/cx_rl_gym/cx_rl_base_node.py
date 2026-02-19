# Copyright (c) 2026 Carologistics
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

import importlib
import os
from threading import Thread

from bondpy import bondpy
from cx_rl_gym import CXRLGym
from lifecycle_msgs.msg import State
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from stable_baselines3.common.monitor import Monitor

# Import SB3 Maskable PPO (or your custom class)


class CXRLBaseNode(LifecycleNode):
    """
    Base class for RL nodes.

    Handles environment and general RL setup, as well as
    Lifecycle and Bond features.
    """

    def __init__(self, node_name='cx_rl_node'):
        super().__init__(node_name)

        # Declare generic parameters for environment, model, training
        self.declare_parameters(
            namespace='',
            parameters=[
                ('storage_dir', '/tmp'),
                ('env.entrypoint', 'cx_rl_gym.cx_rl_gym.CXRLGym'),
                ('rl_mode', 'TRAINING'),
            ],
        )

        self.shutdown_flag = False
        self.env = None
        self.model = None
        self.autostart_group = ReentrantCallbackGroup()
        self.autostart_timer = self.create_timer(
            0.0, self.autostart_callback, callback_group=self.autostart_group
        )

        self.register_rcl_preshutdown_callback()

    def register_rcl_preshutdown_callback(self):
        context = self.context
        context.on_shutdown(self.on_rcl_preshutdown)

    def run_cleanups(self):
        state = self._state_machine.current_state[0]

        if state == State.PRIMARY_STATE_ACTIVE:
            self.trigger_deactivate()
        state = self._state_machine.current_state[0]

        if state == State.PRIMARY_STATE_INACTIVE:
            self.trigger_cleanup()
        state = self._state_machine.current_state[0]

        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.trigger_shutdown()

    def autostart_callback(self):
        self.autostart_timer.cancel()
        self.get_logger().info(f'Auto-starting node: {self.get_name()}')

        configure_result = self.trigger_configure()
        if configure_result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error(f'Auto-starting node {self.get_name()} failed to configure!')
            return

        activate_result = self.trigger_activate()
        if activate_result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error(f'Auto-starting node {self.get_name()} failed to activate!')
            return

    def on_configure(self, state):
        try:
            self.set_dirs()
            self.bond_heartbeat_period = self.get_parameter_or(
                'bond_heartbeat_period',
                rclpy.Parameter('bond_heartbeat_period', rclpy.Parameter.Type.DOUBLE, 0.0),
            ).value
        except Exception as e:
            self.get_logger().error(f'Failed to configure: {e}')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating RL node')

        try:
            self.set_env()
            self.set_model()
            if self.get_parameter('rl_mode').value.upper() == 'TRAINING':
                self.training_thread = Thread(target=self.run_training, daemon=True)
                self.training_thread.start()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to activate: {e}')
            return TransitionCallbackReturn.FAILURE

        self.create_bond()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        visited = set()
        env = self.env

        # clean up recursively, in case env is wrapped
        while env is not None and id(env) not in visited:
            visited.add(id(env))

            if isinstance(env, CXRLGym):
                env.shutdown = True
                break
            env = getattr(env, 'env', None)

        if hasattr(self, 'training_thread'):
            self.training_thread.join(timeout=2.0)

        self.destroy_bond()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.env = None
        self.model = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    def set_dirs(self):
        storage_path = self.get_parameter('storage_dir').value
        self.save_dir = os.path.join(storage_path, 'cx_rl_data', 'trained_agents')
        self.log_dir = os.path.join(storage_path, 'cx_rl_data', 'logs')
        self.checkpoint_dir = os.path.join(storage_path, 'checkpoint_agents')
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.checkpoint_dir, exist_ok=True)

    def set_env(self):
        """Instantiate the environment from the entrypoint parameter."""
        mod_name, attr_name = self.get_parameter('env.entrypoint').value.split(':')
        mod = importlib.import_module(mod_name)
        env_class = getattr(mod, attr_name)
        self.env = Monitor(
            env_class(
                self,
                self.get_parameter('rl_mode').value.upper(),
            )
        )
        return self.env

    def set_model(self):
        """
        Load the model on activation and store it in self.model.

        Implemented in subclass.
        """
        raise NotImplementedError('Subclasses must implement set_model()')

    def run_training(self):
        """
        Run the training loop in a separate thread if rl_mode is 'TRAINING'.

        Implemented in subclass.
        """
        raise NotImplementedError('Subclasses must implement run_training()')

    def create_bond(self):
        if self.bond_heartbeat_period > 0.0:
            self.get_logger().info(f'Creating bond ({self.get_name()}) to lifecycle manager')
            self.bond = bondpy.Bond(self.get_name(), node=self)
            self.bond.start()
            self.bond.set_heartbeat_period(self.bond_heartbeat_period)
            self.bond.set_heartbeat_timeout(4.0)

    def destroy_bond(self):
        if getattr(self, 'bond', None):
            self.get_logger().info(f'Destroying bond ({self.get_name()})')
            self.bond.break_bond()
            self.bond = None

    def on_rcl_preshutdown(self):
        # Best-effort cleanup
        self.run_cleanups()

        # Break bond cleanly
        self.destroy_bond()
