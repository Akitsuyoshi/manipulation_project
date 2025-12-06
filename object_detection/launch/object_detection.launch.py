from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='static_transform_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]),
        Node(
            package='object_detection',
            executable='object_detection',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('object_detection'), 'rviz', 'object_detection.rviz')],
            parameters=[{
                'use_sim_time': True,
            }]),
    ])