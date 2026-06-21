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

import datetime
from pathlib import Path

from cx_rl_gym.cx_rl_base_node import CXRLBaseNode

# Import SB3 Maskable PPO (or your custom class)
from cx_rl_multi_robot_mppo.MultiRobotMaskablePPO import MultiRobotMaskablePPO
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from stable_baselines3.common.callbacks import CheckpointCallback, StopTrainingOnMaxEpisodes
from stable_baselines3.common.logger import configure


class CXRLMaskablePPONode(CXRLBaseNode):
    """Specific RL node using MultiRobotMaskablePPO."""

    def __init__(self, node_name='cx_rl_node'):
        super().__init__(node_name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_name', rclpy.Parameter.Type.STRING),
                ('training.retraining', rclpy.Parameter.Type.BOOL),
                ('training.max_episodes', rclpy.Parameter.Type.INTEGER),
                ('model.learning_rate', rclpy.Parameter.Type.DOUBLE),
                ('model.gamma', rclpy.Parameter.Type.DOUBLE),
                ('model.gae_lambda', rclpy.Parameter.Type.DOUBLE),
                ('model.ent_coef', rclpy.Parameter.Type.DOUBLE),
                ('model.vf_coef', rclpy.Parameter.Type.DOUBLE),
                ('model.max_grad_norm', rclpy.Parameter.Type.DOUBLE),
                ('model.batch_size', rclpy.Parameter.Type.INTEGER),
                ('model.n_steps', rclpy.Parameter.Type.INTEGER),
                ('model.seed', rclpy.Parameter.Type.INTEGER),
                ('model.verbose', rclpy.Parameter.Type.INTEGER),
                ('model.n_robots', rclpy.Parameter.Type.INTEGER),
                ('model.wait_for_all_robots', rclpy.Parameter.Type.BOOL),
                ('training.timesteps', rclpy.Parameter.Type.INTEGER),
                ('training.checkpoint_freq', rclpy.Parameter.Type.INTEGER),
            ],
        )

    def set_model(self):
        rl_mode = self.get_parameter('rl_mode').value.upper()
        sb3_logger = configure(self.log_dir, ['stdout', 'csv', 'log'])
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
        self.env.env.set_rl_model(self.model)
        self.model.set_logger(sb3_logger)
        return self.model

    def create_new_model(self) -> MultiRobotMaskablePPO:
        return MultiRobotMaskablePPO(
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
            n_robots=self.get_parameter('model.n_robots').value,
            time_based=False,
            n_time=300,
            deadzone=5,
            wait_for_all_robots=self.get_parameter('model.wait_for_all_robots').value,
        )

    def load_model(self) -> MultiRobotMaskablePPO:
        model_name = str(self.get_parameter('model_name').value)
        agent_path = Path(self.save_dir) / f'latest_{model_name}.zip'
        if not agent_path.exists():
            agent_path = Path(self.save_dir) / f'{model_name}.zip'
        return MultiRobotMaskablePPO.load(str(agent_path), env=self.env)

    def _update_latest_symlink(self, target: Path, link: Path):
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(target.name)

    def run_training(self):
        model_name = str(self.get_parameter('model_name').value)
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        run_id = f'{model_name}_{timestamp}'

        run_ckpt_dir = Path(self.checkpoint_dir) / run_id
        run_ckpt_dir.mkdir(parents=True, exist_ok=True)

        callback_max_episodes = StopTrainingOnMaxEpisodes(
            max_episodes=self.get_parameter('training.max_episodes').value, verbose=1
        )

        checkpoint_callback = CheckpointCallback(
            save_freq=self.get_parameter('training.checkpoint_freq').value,
            save_path=str(run_ckpt_dir),
            name_prefix=model_name,
        )

        self._update_latest_symlink(
            run_ckpt_dir, Path(self.checkpoint_dir) / f'latest_{model_name}'
        )

        self.model.learn(
            total_timesteps=self.get_parameter('training.timesteps').value,
            callback=[callback_max_episodes, checkpoint_callback],
            log_interval=1,
        )

        final_path = Path(self.save_dir) / f'{run_id}.zip'
        self.model.save(str(final_path.with_suffix('')))
        self._update_latest_symlink(
            final_path, Path(self.save_dir) / f'latest_{model_name}.zip'
        )

        self.env.env.on_training_end()
        self.get_logger().info('Finished training')
        self.on_rcl_preshutdown()

        # Drop extra reference to parameter event publisher.
        # It will be destroyed with other publishers below.
        self._parameter_event_publisher = None
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = CXRLMaskablePPONode()
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
