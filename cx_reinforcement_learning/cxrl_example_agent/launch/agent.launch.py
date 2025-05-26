import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_with_context(context, *args, **kwargs):
    blocksworld_dir = get_package_share_directory('cxrl_example_agent')
    manager_config = LaunchConfiguration("manager_config")
    rl_config = os.path.join(blocksworld_dir, 'params', 'training-config.yaml')
    log_level = LaunchConfiguration('log_level')
    manager_config_file = os.path.join(blocksworld_dir, "params", manager_config.perform(context))
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(blocksworld_dir + "/domain.pddl"),
        description='PDDL Model file')


    cx_node = Node(
        package='cx_bringup',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            manager_config_file,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    cxrl_node = Node(
        package='cxrl_mrmppo',
        executable='cxrl_node',
        namespace='cxrl_node',
        name='blocksworld_rl_node',
        output='screen',
        emulate_tty=True,
        parameters= [rl_config]
    )

    return [cx_node, cxrl_node]

def generate_launch_description():

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value='info',
        description="Logging level for cx_node executable",
    )
    declare_manager_config = DeclareLaunchArgument(
        "manager_config",
        default_value="agent.yaml",
        description="Name of the CLIPS environment manager configuration",
    )

    ld = LaunchDescription()

    ld.add_action(declare_log_level_)
    ld.add_action(declare_manager_config)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld