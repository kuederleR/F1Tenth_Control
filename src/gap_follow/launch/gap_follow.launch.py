from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow_node',
            parameters=[params_file]
        ),
        Node(
            package='gap_follow',
            executable='emergency_brake',
            name='emergency_brake_node',
            parameters=[params_file]
        )
    ])