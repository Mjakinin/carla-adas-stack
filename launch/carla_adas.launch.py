import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('carla_adas_stack')
    config_file = os.path.join(pkg_dir, 'config', 'adas_params.yaml')

    spawn_index_arg = DeclareLaunchArgument(
        'spawn_index',
        default_value='0',
        description='CARLA map spawn index for ego vehicle'
    )
    enable_overtake_arg = DeclareLaunchArgument(
        'enable_overtaking',
        default_value='false',
        description='Enable dynamic S-curve overtaking maneuver in Frenet Planner'
    )

    return LaunchDescription([
        spawn_index_arg,
        enable_overtake_arg,

        # 1. CARLA ROS 2 Interface Node (Spawner & Bridge)
        Node(
            package='carla_adas_stack',
            executable='carla_interface_node',
            name='carla_interface_node',
            output='screen',
            parameters=[
                config_file,
                {'spawn_index': LaunchConfiguration('spawn_index')}
            ]
        ),

        # 2. EKF State Estimator Node
        Node(
            package='carla_adas_stack',
            executable='state_estimator_node',
            name='state_estimator_node',
            output='screen',
            parameters=[config_file]
        ),

        # 3. Frenet Trajectory Planner Node
        Node(
            package='carla_adas_stack',
            executable='frenet_planner_node',
            name='frenet_planner_node',
            output='screen',
            parameters=[
                config_file,
                {'enable_overtaking': LaunchConfiguration('enable_overtaking')}
            ]
        ),

        # 4. Stanley Path Tracking Controller Node
        # Remap output to ackermann_cmd_raw so AEB supervisor can validate it
        Node(
            package='carla_adas_stack',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/carla/ego_vehicle/ackermann_cmd', '/carla/ego_vehicle/ackermann_cmd_raw')
            ]
        ),

        # 5. AEB & TTC Safety Supervisor Node
        # Receives ackermann_cmd_raw, supervises, and publishes the safe ackermann_cmd
        Node(
            package='carla_adas_stack',
            executable='aeb_node',
            name='aeb_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
