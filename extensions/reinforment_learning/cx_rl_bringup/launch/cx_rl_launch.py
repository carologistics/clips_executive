#!/usr/bin/env/python3
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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from rclpy.logging import get_logging_directory


def generate_launch_description():
    # -----------------------------
    # Launch arguments
    # -----------------------------
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='namepace for the nodes',
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for cx_node executable',
    )

    declare_manager_config = DeclareLaunchArgument(
        'manager_config',
        default_value='rl_agent.yaml',
        description='Name of the CLIPS environment manager configuration',
    )

    declare_training_config = DeclareLaunchArgument(
        'training_config',
        default_value='rl_node_config.yaml',
        description='Name of the RL training configuration',
    )

    declare_rl_mode = DeclareLaunchArgument(
        'rl_mode',
        default_value='EXECUTION',
        description='Mode for RL',
    )

    declare_storage_dir = DeclareLaunchArgument(
        'storage_dir',
        default_value=PathJoinSubstitution([get_logging_directory(), 'cx_rl_mppo_agents']),
        description='Absolute path, where trained agents and checkpoints are stored',
    )

    # TODO: This should be an error, but is not supported as action yet
    invalid_rl_mode = LogInfo(
        condition=IfCondition(
            PythonExpression(
                ['"', LaunchConfiguration('rl_mode'), '" not in ["TRAINING", "EXECUTION"]']
            )
        ),
        msg=[
            'Invalid value for rl_mode: ',
            LaunchConfiguration('rl_mode'),
            '. Allowed values are TRAINING or EXECUTION.',
        ],
    )
    shutdown_on_invalid = Shutdown(
        condition=IfCondition(
            PythonExpression(
                ['"', LaunchConfiguration('rl_mode'), '" not in ["TRAINING", "EXECUTION"]']
            )
        )
    )

    # LaunchConfigurations
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')
    manager_config = LaunchConfiguration('manager_config')
    training_config = LaunchConfiguration('training_config')
    storage_dir = LaunchConfiguration('storage_dir')
    rl_mode = LaunchConfiguration('rl_mode')

    # -----------------------------
    # Paths to packages and files
    # -----------------------------
    cx_rl_bringup_dir = get_package_share_directory('cx_rl_bringup')
    cx_bringup_dir = get_package_share_directory('cx_bringup')

    rl_config_file = PathJoinSubstitution([cx_rl_bringup_dir, 'params', training_config])
    cx_launch_file = PathJoinSubstitution([cx_bringup_dir, 'launch', 'cx_launch.py'])

    # -----------------------------
    # Nodes and included launches
    # -----------------------------
    cx_rl_node = Node(
        package='cx_rl_multi_robot_mppo',
        executable='cx_rl_mppo_node',
        namespace=namespace,
        name='cx_rl_node',
        output='screen',
        emulate_tty=True,
        parameters=[rl_config_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    include_cx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cx_launch_file),
        launch_arguments={
            'package': 'cx_rl_bringup',
            'manager_config': manager_config,
        }.items(),
    )

    # -----------------------------
    # Build LaunchDescription
    # -----------------------------
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_namespace)
    ld.add_action(declare_log_level)
    ld.add_action(declare_manager_config)
    ld.add_action(declare_training_config)
    ld.add_action(declare_storage_dir)
    ld.add_action(declare_rl_mode)
    ld.add_action(invalid_rl_mode)
    ld.add_action(shutdown_on_invalid)
    # pass down parameters to both nodes (even through nested launch)
    ld.add_action(SetParameter(name='storage_dir', value=storage_dir))
    ld.add_action(SetParameter(name='rl_mode', value=rl_mode))
    ld.add_action(cx_rl_node)
    ld.add_action(include_cx_launch)

    return ld
