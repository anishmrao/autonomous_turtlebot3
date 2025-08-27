from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    selection_mode = LaunchConfiguration('selection_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_selection_mode = DeclareLaunchArgument(
        'selection_mode',
        default_value='closest',
        description="Explorer goal selection mode: 'closest' or 'largest'")
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    explorer_node = Node(
        package='tb3_explorer',
        executable='explorer',
        name='simple_frontier_explorer',
        output='screen',
        parameters=[
            {'selection_mode': selection_mode},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_selection_mode,
        declare_use_sim_time,
        explorer_node,
    ])