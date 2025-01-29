from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージ共有パスの取得
    urdf_tutorial_path = FindPackageShare('car_like_mobile_robot_description')
    default_model_path = PathJoinSubstitution([urdf_tutorial_path, 'urdf', 'car_like_mobile_robot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'urdf.rviz'])

    # Launch引数の宣言
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to the RViz2 configuration file'
    )
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to the robot Xacro file'
    )

    # Robot State Publisherノード
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }],
        output='screen'
    )

    # Joint State Publisherノード
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui' if LaunchConfiguration('gui') == 'true' else 'joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2ノード
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    # LaunchDescriptionの作成
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
