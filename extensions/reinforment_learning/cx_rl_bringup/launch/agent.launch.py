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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_with_context(context, *args, **kwargs):
    cx_rl_bringup_dir = get_package_share_directory('cx_rl_bringup')
    cx_bringup_dir = get_package_share_directory('cx_bringup')
    manager_config = LaunchConfiguration('manager_config')
    training_config = LaunchConfiguration('training_config')
    rl_config = os.path.join(cx_rl_bringup_dir, 'params', training_config.perform(context))
    log_level = LaunchConfiguration('log_level')
    # also launch the pddl_manager
    launch_cx = os.path.join(cx_bringup_dir, 'launch', 'cx_launch.py')

    cx_rl_node = Node(
        package='cx_rl_multi_robot_mppo',
        executable='cx_rl_node',
        namespace='cx_rl_node',
        name='cx_rl_bringup_rl_node',
        output='screen',
        emulate_tty=True,
        parameters=[rl_config],
    )

    return [
        cx_rl_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_cx),
            launch_arguments={
                'package': 'cx_rl_bringup',
                'manager_config': manager_config.perform(context),
                'log_level': log_level,
            }.items(),
        ),
    ]


def generate_launch_description():

    LaunchConfiguration('model_file')

    declare_log_level_ = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for cx_node executable',
    )
    declare_manager_config = DeclareLaunchArgument(
        'manager_config',
        default_value='training_agent.yaml',
        description='Name of the CLIPS environment manager configuration',
    )
    declare_training_config = DeclareLaunchArgument(
        'training_config',
        default_value='training_config.yaml',
        description='Name of the RL training configuration',
    )

    ld = LaunchDescription()

    ld.add_action(declare_log_level_)
    ld.add_action(declare_manager_config)
    ld.add_action(declare_training_config)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
