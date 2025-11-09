from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'params.yaml'
    )

    additional_gap_follow_params = {
        'deadman_enable': True
    }

    f1tenth_stack_pkg = 'f1tenth_stack'
    f1tenth_stack_dir = get_package_share_directory(f1tenth_stack_pkg)

    f1tenth_stack_launch_file = os.path.join(
        f1tenth_stack_dir,
        'launch/'
        'bringup_launch.py'
    )

    f1tenth_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f1tenth_stack_launch_file)
    )

    return LaunchDescription([
        f1tenth_stack_launch,

        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow_node',
            parameters=[params_file, additional_gap_follow_params]
        ),
    ])