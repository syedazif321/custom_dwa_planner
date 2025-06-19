from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    param_path = PathJoinSubstitution([
        FindPackageShare("dwa_planner"), "config", "dwa_params.yaml"
    ])
    rviz_path = PathJoinSubstitution([
        FindPackageShare("dwa_planner"), "rviz", "dwa_planner_config.rviz"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=param_path),
        Node(
            package='dwa_planner',
            executable='dwa_planner_node',
            name='dwa_planner_node',
            parameters=[LaunchConfiguration('params')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
    ])