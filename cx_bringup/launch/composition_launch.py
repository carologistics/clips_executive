#!/usr/bin/env python3
# Copyright (c) 2025 Carologistics
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
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from rclpy.logging import get_logger


def launch_with_context(context, *args, **kwargs):
    bringup_dir = get_package_share_directory("cx_bringup")

    namespace = LaunchConfiguration("namespace")
    manager_config = LaunchConfiguration("manager_config")
    manager_config_file = os.path.join(bringup_dir, "params", manager_config.perform(context))
    # re-issue warning as it is not colored otherwise ...
    if not os.path.isfile(manager_config_file):
        logger = get_logger("cx_bringup_launch")
        logger.warning(f"Parameter file path is not a file: {manager_config_file}")

    LaunchConfiguration("log_level")
    container = ComposableNodeContainer(
        name="clips_executive_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        emulate_tty=True,
        output="screen",  # both
    )
    load_composable_nodes = LoadComposableNodes(
        target_container="clips_executive_container",
        composable_node_descriptions=[
            ComposableNode(
                package="cx_clips_env_manager",
                plugin="cx::CLIPSEnvManager",
                name="clips_manager",
                namespace=namespace,
                parameters=[manager_config_file, {"namespace": namespace, "autostart_node": True}],
            ),
        ],
    )
    return [container, load_composable_nodes]


def generate_launch_description():

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_manager_config = DeclareLaunchArgument(
        "manager_config",
        default_value="plugin_examples/file_load.yaml",
        description="Name of the CLIPS environment manager configuration",
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_manager_config)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
