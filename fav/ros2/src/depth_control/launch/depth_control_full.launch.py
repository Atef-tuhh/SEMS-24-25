from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    # Fahrzeugname als Namespace
    vehicle_name_arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(vehicle_name_arg)

    # Option, ob Simulationszeit verwendet wird
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value='false')
    launch_description.add_action(use_sim_time_arg)

    # Standard-Parameterdatei für den Controller
    package_path = get_package_share_path('depth_control')
    param_file_path = str(package_path / 'config/controller_params.yaml')
    controller_param_arg = DeclareLaunchArgument('controller_config_file',
                                                 default_value=param_file_path)
    launch_description.add_action(controller_param_arg)

    # Gruppierte Nodes unter dem Namespace
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),

        # Node für Depth Calculator
        Node(
            executable='depth_calculator.py',
            package='depth_control',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),

        # Node für Depth Controller
        Node(
            executable='depth_controller.py',
            package='depth_control',
            parameters=[
                LaunchConfiguration('controller_config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),

        # Node für Depth Setpoint
        Node(
            executable='depth_setpoint.py',
            package='depth_control',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
    launch_description.add_action(group)

    return launch_description
