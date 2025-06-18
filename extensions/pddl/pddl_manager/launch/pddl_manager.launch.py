#!/bin/env/python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rclpy.logging import get_logger
from launch.actions import TimerAction, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def launch_with_context(context, *args, **kwargs):
    bringup_dir = get_package_share_directory("pddl_manager")

    namespace = LaunchConfiguration("namespace")
    config = LaunchConfiguration("config")
    config_file = os.path.join(bringup_dir, "", config.perform(context))
    # re-issue warning as it is not colored otherwise ...
    if not os.path.isfile(config_file):
        logger = get_logger("cx_bringup_launch")
        logger.warning(f"Parameter file path is not a file: {config_file}")

    log_level = LaunchConfiguration("log_level")
    pddl_manager_node = LifecycleNode(
            package='pddl_manager',
            executable='pddl_manager',
            name='pddl_manager',
            namespace='',
            emulate_tty=True,
            output='screen',
            parameters=[config_file],
        )
    return [pddl_manager_node
            ]


def generate_launch_description():

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_config = DeclareLaunchArgument(
        "config",
        default_value="params/pddl_manager.yaml",
        description="Name of the configuration file.",
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_config)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
