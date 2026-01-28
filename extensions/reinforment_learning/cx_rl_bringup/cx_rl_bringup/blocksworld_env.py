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

from cx_rl_gym.cx_rl_gym import CXRLGym
from rclpy.node import Node


class BlocksworldEnv(CXRLGym):

    def __init__(self, node: Node, mode: str, number_robots: int):
        self.reward_in_episode = 0
        self.episode_number = 0
        super().__init__(node, mode, number_robots)

    def step(self, action):
        with open('cxrl-bw-log-episode-reward.txt', 'a+') as f:
            f.write(f'{self.action_dict[action]} \n')
        state, reward, done, truncated, info = super().step(action)
        self.reward_in_episode += reward
        return state, reward, done, truncated, info

    def reset(self, seed: int = None, options: dict[str, any] = None):
        with open('cxrl-bw-log-episode-reward.txt', 'a+') as f:
            f.write(f'{self.reward_in_episode} \n')
        self.node.get_logger().info(f'Episode {self.episode_number}.')
        self.episode_number += 1
        self.reward_in_episode = 0
        return super().reset(seed=seed)

    def render(self):
        pass
