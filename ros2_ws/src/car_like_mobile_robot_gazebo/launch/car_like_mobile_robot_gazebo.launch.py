import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument("sim", default_value="True")
    sim = LaunchConfiguration("sim")

    # Launch Arguments
    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='empty.sdf',
                          description='Gz sim World'),
           ]
    )

    car_like_mobile_robot_description_path = os.path.join(
        get_package_share_directory("car_like_mobile_robot_description"))
    car_like_mobile_robot_gazebo_path = os.path.join(
        get_package_share_directory("car_like_mobile_robot_gazebo"))

      # set gazebo resource path
    # gazeboのmodel(SDF)のリソースのパスを通します。(URDF内でmodel://hcr_description/meshes/...とアクセスしているため)
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(car_like_mobile_robot_gazebo_path, 'worlds'), ':' +
            str(Path(car_like_mobile_robot_description_path).parent.resolve())
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                            # ' -v 4',   #ログを表示する
                            ' -r']     #起動時にシミュレーションを開始する
            )
        ]
    )


   
    # print(os.path.join(car_like_mobile_robot_bringup_path, 'worlds'), ':' +
    #         str(Path(car_like_mobile_robot_description_path).parent.resolve()))
    # print(f"GZ_SIM_RESOURCE_PATH: {os.getenv('GZ_SIM_RESOURCE_PATH')}")
    # # xacro to urdf
    xacro_file = os.path.join(car_like_mobile_robot_description_path,
                              'urdf',
                              'car_like_mobile_robot.urdf.xacro')
    xacro_doc = xacro.process_file(xacro_file)
    robot_desc = xacro_doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    # #robot_state_publisherの起動
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # # ロボットのスポーン
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name','car_like_mobile_robot'],
    )

    # #joint_state_broadcasterの起動
    # #コマンド「ros2 control load_controller --set-state active joint_state_broadcaster」と同義
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    load_front_steering_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'front_steering_position_controller'],
        output='screen'
    )
    load_front_wheel_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'front_wheel_effort_controller'],
        output='screen'
    )
    load_rear_wheel_speed_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'rear_wheel_speed_controller'],
        output='screen'
    )

    # # Ros2とGazeboのBridge
    bridge_params = os.path.join(car_like_mobile_robot_gazebo_path,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        #gz_spawn_entityの実行が終わったらload_joint_state_controllerを起動
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_front_steering_position_controller,
                        load_rear_wheel_speed_controller,
                        load_front_wheel_effort_controller],
            )   
        ),
        sim_arg,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        ros_gz_bridge,

    ])