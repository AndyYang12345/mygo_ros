import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('mygo_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'arm_new', '-topic', 'robot_description']
    )

    tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        spawn_entity,
        tf_footprint
    ])
