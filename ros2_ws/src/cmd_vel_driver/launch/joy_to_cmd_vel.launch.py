import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    cmd_vel_driver_path = os.path.join(
        get_package_share_directory("cmd_vel_driver"))
    cmd_vel_driver_config_path = os.path.join(
        cmd_vel_driver_path, 'config', 'joy_to_cmd_vel_config.yaml')
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_to_cmd_vel_node = Node(
            package='cmd_vel_driver',
            executable='joy_to_cmd_vel',
            name='joy_to_cmd_vel',
            output='screen',
            parameters=[cmd_vel_driver_config_path]
        )

    return LaunchDescription([
        joy_node,
        joy_to_cmd_vel_node,

    ])
