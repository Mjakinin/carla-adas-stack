import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('carla_adas_stack')
    config_file = os.path.join(pkg_dir, 'config', 'adas_params.yaml')

    return LaunchDescription([
        Node(
            package='carla_adas_stack',
            executable='carla_interface_node',
            name='carla_interface_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
