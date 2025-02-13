#!/usr/bin/env python3
import os
import yaml


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from car_like_mobile_robot_bringup import generate_bezier_box


def generate_launch_description():
    # 各パッケージのディレクトリを取得
    car_like_mobile_robot_gazebo_share_dir = os.path.join(get_package_share_directory('car_like_mobile_robot_gazebo'))
    car_like_mobile_robot_bringup_share_dir = os.path.join(get_package_share_directory('car_like_mobile_robot_bringup'))
        
    #共通パラメータファイルのパス
    common_param_file_path = os.path.join(car_like_mobile_robot_bringup_share_dir, 'config', 'common_param.yaml')
    # YAMLファイルを読み込み、辞書型に変換
    with open(common_param_file_path, 'r') as file:
        params = yaml.safe_load(file)

    points = params.get('ros__parameters', {}).get('control_points', [])
    print("Loaded points:", points)

    #ワールドファイルの作成
    world_file_name = 'bezier_box.sdf'
    world_file_path = os.path.join(car_like_mobile_robot_gazebo_share_dir, 'worlds', world_file_name)
    generate_bezier_box.create_world(points,filename=world_file_path)

    car_like_mobile_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            car_like_mobile_robot_gazebo_share_dir, 'launch'), '/car_like_mobile_robot_gazebo.launch.py']),
        launch_arguments={'world': world_file_name}.items()
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
