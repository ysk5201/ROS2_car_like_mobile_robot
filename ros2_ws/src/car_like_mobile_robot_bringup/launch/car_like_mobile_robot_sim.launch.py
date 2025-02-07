#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 各パッケージのディレクトリを取得
    car_like_mobile_robot_gazebo_share_dir = os.path.join(get_package_share_directory('car_like_mobile_robot_gazebo'))
    car_like_mobile_robot_driver_share_dir = os.path.join(get_package_share_directory('car_like_mobile_robot_driver'))


    # IncludeLaunchDescription で他パッケージの launch ファイルを含める
    # include_my_pkg_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(my_pkg_launch_file)
    # )

    car_like_mobile_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            car_like_mobile_robot_gazebo_share_dir, 'launch'), '/car_like_mobile_robot_gazebo.launch.py']),
    )

    state_variable_pub_node = Node(
        package='car_like_mobile_robot_gazebo',
        executable='state_variable_pub',
        name='state_variable_pub',
    )

    driver_node = Node(
        package='car_like_mobile_robot_driver',
        executable='car_like_mobile_robot',
        name='car_like_mobile_robot',
        output='screen',
        prefix='xterm -e' #別ターミナルを起動
    )

    return LaunchDescription([
        car_like_mobile_robot_gazebo,
        state_variable_pub_node,
        driver_node   
    ])
