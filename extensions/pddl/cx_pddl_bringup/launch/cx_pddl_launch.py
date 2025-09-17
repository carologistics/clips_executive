# Copyright (c) 2025 Carologistics
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


def launch_with_context(context, *args, **kwargs):
    get_package_share_directory('cx_pddl_bringup')
    cx_bringup_dir = get_package_share_directory('cx_bringup')
    manager_config = LaunchConfiguration('manager_config')
    log_level = LaunchConfiguration('log_level')

    # also launch the pddl_manager
    pddl_manager_dir = get_package_share_directory('cx_pddl_manager')
    launch_pddl_manager = os.path.join(pddl_manager_dir, 'launch', 'pddl_manager.launch.py')
    launch_cx = os.path.join(cx_bringup_dir, 'launch', 'cx_launch.py')

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_pddl_manager)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_cx),
            launch_arguments={
                'package': 'cx_pddl_bringup',
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
        default_value='pddl_agents/structured_agent.yaml',
        description='Name of the CLIPS environment manager configuration',
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)
    ld.add_action(declare_manager_config)

    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
