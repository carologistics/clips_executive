#!/usr/bin/env python3
# Copyright (c) 2025-2026 Carologistics
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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import LifecycleNode, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -----------------------------
    # Launch arguments
    # -----------------------------
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='namepace for the nodes',
    )

    declare_package = DeclareLaunchArgument(
        'package',
        default_value='cx_pddl_bringup',
        description='The name of package where to look for the manager config',
    )
    declare_manager_config = DeclareLaunchArgument(
        'manager_config',
        default_value='pddl_agents/generic_agent.yaml',
        description='Name of the CLIPS environment manager configuration',
    )

    declare_pddl_manager_config = DeclareLaunchArgument(
        'pddl_manager_config',
        default_value='pddl_manager.yaml',
        description='Name of the PDDL manager configuration',
    )

    declare_pddl_domain = DeclareLaunchArgument(
        'pddl_domain',
        default_value='pddl/domain.pddl',
        description='Path to a PDDL domain file relative to the packge.',
    )

    declare_pddl_problem = DeclareLaunchArgument(
        'pddl_problem',
        default_value='pddl/problem.pddl',
        description='Path to a PDDL problem file relative to the packge.',
    )

    declare_pddl_plan_type = DeclareLaunchArgument(
        'pddl_plan_type',
        default_value='CLASSICAL',
        description='Plan type. Valid options: CLASSICAL, TEMPORAL, STN, '
        'PARTIAL-ORDER, HIERARCHICAL.',
    )
    # TODO: This should be an error, but is not supported as action yet
    invalid_pddl_plan_type = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration('pddl_plan_type'),
                    '" not in ["CLASSICAL", "TEMPORAL", "STN", "PARTIAL-ORDER", "HIERARCHICAL"]',
                ]
            )
        ),
        msg=[
            'Invalid value for pddl_plan_type: ',
            LaunchConfiguration('pddl_plan_type'),
            '. Allowed values are CLASSICAL, TEMPORAL, STN, PARTIAL-ORDERor HIERARCHICAL.',
        ],
    )
    shutdown_on_invalid = Shutdown(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration('pddl_plan_type'),
                    '" not in ["CLASSICAL", "TEMPORAL", "STN", "PARTIAL-ORDER", "HIERARCHICAL"]',
                ]
            )
        )
    )
    package = LaunchConfiguration('package')
    manager_config = LaunchConfiguration('manager_config')
    namespace = LaunchConfiguration('namespace')

    pddl_plan_type = LaunchConfiguration('pddl_plan_type')
    pddl_domain = LaunchConfiguration('pddl_domain')
    pddl_problem = LaunchConfiguration('pddl_problem')
    pddl_manager_config = LaunchConfiguration('pddl_manager_config')

    # Resolve full path to manager config
    pddl_manager_config_file = PathJoinSubstitution(
        [FindPackageShare(package), TextSubstitution(text='params/'), pddl_manager_config]
    )

    pddl_manager_node = LifecycleNode(
        package='cx_pddl_manager',
        executable='pddl_manager',
        name='pddl_manager',
        namespace=namespace,
        emulate_tty=True,
        output='screen',
        parameters=[pddl_manager_config_file],
    )

    # -----------------------------
    # Paths to other launch file
    # -----------------------------
    cx_launch_file = PathJoinSubstitution(
        [get_package_share_directory('cx_bringup'), 'launch', 'cx_launch.py']
    )

    # -----------------------------
    # Include launch description
    # -----------------------------
    include_cx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cx_launch_file),
        launch_arguments={
            'package': package,
            'manager_config': manager_config,
        }.items(),
    )

    # -----------------------------
    # Build LaunchDescription
    # -----------------------------
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_package)
    ld.add_action(declare_manager_config)
    ld.add_action(declare_pddl_manager_config)
    ld.add_action(declare_pddl_plan_type)
    ld.add_action(declare_pddl_domain)
    ld.add_action(declare_pddl_problem)
    ld.add_action(pddl_manager_node)
    ld.add_action(invalid_pddl_plan_type)
    ld.add_action(shutdown_on_invalid)
    # pass down parameters to CX (even through nested launch)
    ld.add_action(SetParameter(name='pddl.plan_type', value=pddl_plan_type))
    ld.add_action(SetParameter(name='pddl.domain', value=pddl_domain))
    ld.add_action(SetParameter(name='pddl.problem', value=pddl_problem))
    ld.add_action(include_cx_launch)

    return ld
