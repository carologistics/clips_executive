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

import os
import signal
from threading import Thread
import time

from cx_rl_gym.cx_rl_base_node import CXRLBaseNode

# Import SB3 Maskable PPO (or your custom class)
from cx_rl_multi_robot_mppo.MultiRobotMaskablePPO import MultiRobotMaskablePPO
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from stable_baselines3.common.callbacks import CheckpointCallback, StopTrainingOnMaxEpisodes
from stable_baselines3.common.logger import configure


class CXRLMaskablePPONode(CXRLBaseNode):
    """Specific RL node using MultiRobotMaskablePPO."""

    def set_model(self):
        rl_mode = self.get_parameter('rl_mode').value.upper()
        if rl_mode == 'TRAINING':
            if self.get_parameter('training.retraining').value:
                self.get_logger().info('Retraining existing agent...')
                self.model = self.load_model()
            else:
                self.get_logger().info('Creating new agent...')
                self.model = self.create_new_model()
        elif rl_mode == 'EXECUTION':
            self.get_logger().info('Executing existing agent...')
            self.model = self.load_model()
        return self.model

    def create_new_model(self) -> MultiRobotMaskablePPO:
        self.model = MultiRobotMaskablePPO(
            policy=MaskableActorCriticPolicy,
            env=self.env,
            learning_rate=self.get_parameter('model.learning_rate').value,
            gamma=self.get_parameter('model.gamma').value,
            gae_lambda=self.get_parameter('model.gae_lambda').value,
            ent_coef=self.get_parameter('model.ent_coef').value,
            vf_coef=self.get_parameter('model.vf_coef').value,
            max_grad_norm=self.get_parameter('model.max_grad_norm').value,
            batch_size=self.get_parameter('model.batch_size').value,
            n_steps=self.get_parameter('model.n_steps').value,
            seed=self.get_parameter('model.seed').value,
            verbose=self.get_parameter('model.verbose').value,
            n_robots=self.get_parameter('number_of_robots').value,
            time_based=False,
            n_time=300,
            deadzone=5,
            wait_for_all_robots=self.get_parameter('model.wait_for_all_robots').value,
        )
        sb3_logger = configure(self.log_dir, ['stdout', 'csv', 'log'])
        self.model.set_logger(sb3_logger)
        self.env.env.set_rl_model(self.model)
        return self.model

    def load_model(self) -> MultiRobotMaskablePPO:
        agent_path = os.path.join(
            self.save_dir, str(self.get_parameter('agent_name').value) + '.zip'
        )
        self.model = MultiRobotMaskablePPO.load(agent_path, env=self.env)
        self.env.env.set_rl_model(self.model)
        self.get_logger().info('Agent loaded')
        return self.model


def main(args=None):
    rclpy.init()
    node = CXRLMaskablePPONode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    Thread(target=executor.spin).start()
    signal.signal(signal.SIGINT, node.shutdown)

    node.get_logger().info('Creating environment')
    node.create_env()

    model = node.set_model()

    if node.get_parameter('rl_mode').value.upper() == 'TRAINING':
        callback_max_episodes = StopTrainingOnMaxEpisodes(
            max_episodes=node.get_parameter('training.max_episodes').value, verbose=1
        )
        checkpoint_callback = CheckpointCallback(save_freq=200, save_path=node.checkpoint_dir)

        model.learn(
            total_timesteps=node.get_parameter('training.timesteps').value,
            callback=[callback_max_episodes, checkpoint_callback],
            log_interval=1,
        )
        model.save(os.path.join(node.save_dir, str(node.get_parameter('agent_name').value)))
        node.env.env.on_training_end()
        node.get_logger().info('Finished training, closing node')
    else:
        while not node.shutdown_flag:
            time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()
