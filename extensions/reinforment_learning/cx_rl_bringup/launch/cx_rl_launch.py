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


import datetime
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
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
        output='both',
        emulate_tty=True,
        parameters=[rl_config_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    run_state = {'dir': None}

    def setup_run_log_dir(context, *args, **kwargs):
        storage = context.perform_substitution(storage_dir)
        model_name = 'cx_rl_agent'
        ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        run_id = f'{model_name}_{ts}'
        run_dir = os.path.join(storage, 'cx_rl_data', 'logs', run_id)
        os.makedirs(run_dir, exist_ok=True)
        latest = os.path.join(storage, 'cx_rl_data', 'logs', f'latest_{model_name}')
        if os.path.islink(latest) or os.path.exists(latest):
            os.unlink(latest)
        os.symlink(run_id, latest)
        run_state['dir'] = run_dir
        return [
            SetEnvironmentVariable('ROS_LOG_DIR', run_dir),
            SetEnvironmentVariable('CX_RL_RUN_ID', run_id),
        ]

    setup_logs = OpaqueFunction(function=setup_run_log_dir)

    def rename_unix_ms_logs(event, context):
        run_dir = run_state.get('dir')
        if not run_dir or not os.path.isdir(run_dir):
            return
        pattern = re.compile(r'^(.+?)_(\d+)_(\d{13})\.log$')
        for fname in os.listdir(run_dir):
            m = pattern.match(fname)
            if not m:
                continue
            name, _pid, ms = m.group(1), m.group(2), int(m.group(3))
            dt = datetime.datetime.fromtimestamp(ms / 1000).strftime('%Y-%m-%d_%H-%M-%S')
            new_name = f'{name}_{dt}.log'
            src = os.path.join(run_dir, fname)
            dst = os.path.join(run_dir, new_name)
            if os.path.exists(dst):
                continue
            try:
                os.rename(src, dst)
            except OSError:
                pass

    rename_logs = RegisterEventHandler(OnShutdown(on_shutdown=rename_unix_ms_logs))

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
    ld.add_action(setup_logs)
    ld.add_action(rename_logs)
    ld.add_action(cx_rl_node)
    ld.add_action(include_cx_launch)

    return ld
