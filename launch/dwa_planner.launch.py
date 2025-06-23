from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch your DWA planner node
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner_node',
            name='dwa_planner',
            output='screen',
            parameters=['config/params.yaml']
        ),

        # Launch RViz with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('custom_dwa_planner'),
                    'rviz',
                    'dwa_markers.rviz'
                ])
            ],
            output='screen'
        )
    ])
