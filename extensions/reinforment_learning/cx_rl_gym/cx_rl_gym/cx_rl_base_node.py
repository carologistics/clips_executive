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

import rclpy
from rclpy.node import Node
from stable_baselines3.common.monitor import Monitor

# Import SB3 Maskable PPO (or your custom class)


class CXRLBaseNode(Node):
    """Base class for RL nodes: handles environment and general RL setup."""

    def __init__(self, node_name='cx_rl_node'):
        super().__init__(node_name)

        # Declare generic parameters for environment, model, training
        self.declare_parameters(
            namespace='',
            parameters=[
                ('storage_dir', rclpy.Parameter.Type.STRING),
                ('env.entrypoint', rclpy.Parameter.Type.STRING),
                ('rl_mode', rclpy.Parameter.Type.STRING),
                ('number_of_robots', rclpy.Parameter.Type.INTEGER),
            ],
        )

        self.shutdown_flag = False
        self.env = None
        self.model = None
        self.set_dirs()
        self.get_logger().info(f'{node_name} initialised')

    def set_dirs(self):
        storage_path = self.get_parameter('storage_dir').value
        self.get_logger().info(f"{storage_path}")
        self.save_dir = os.path.join(storage_path, 'cx_rl_multi_robot_mppo', 'trained_agents')
        self.log_dir = os.path.join(storage_path, 'cx_rl_multi_robot_mppo', 'logs')
        self.checkpoint_dir = os.path.join(storage_path, 'checkpoint_agents')
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.checkpoint_dir, exist_ok=True)

    def create_env(self):
        """Instantiate the environment from the entrypoint parameter."""
        mod_name, attr_name = self.get_parameter('env.entrypoint').value.split(':')
        mod = importlib.import_module(mod_name)
        env_class = getattr(mod, attr_name)
        self.env = Monitor(
            env_class(
                self,
                self.get_parameter('rl_mode').value.upper(),
                self.get_parameter('number_of_robots').value,
            )
        )
        return self.env

    def shutdown(self, sig, frame):
        self.shutdown_flag = True
        if self.env:
            self.env.shutdown = True
        if self.model:
            self.model.shutdown = True

    def set_model(self):
        """Implemented in subclass: create or load a model."""
        raise NotImplementedError('Subclasses must implement set_model()')
