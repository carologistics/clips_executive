import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_with_context(context, *args, **kwargs):
    # configure and launc the CX node
    example_agent_dir = get_package_share_directory('pddl_agent_example')
    cx_config_file = os.path.join(example_agent_dir, "params", "config.yaml")

    log_level = LaunchConfiguration('log_level')
    cx_node = Node(
        package='cx_clips_env_manager',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            cx_config_file,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # launch the pddl_manager
    pddl_manager_dir = get_package_share_directory('pddl_manager')
    launch_pddl_manager = os.path.join(pddl_manager_dir, 'launch', 'pddl_manager.launch.py')

    return [cx_node, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_pddl_manager)
        ),]

def generate_launch_description():
    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value='info',
    )
    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()
    ld.add_action(declare_log_level_)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
