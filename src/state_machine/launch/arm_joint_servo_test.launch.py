from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )

    step_rad_arg = DeclareLaunchArgument(
        "step_rad",
        default_value="0.1",
        description="Angle step (rad) per joystick update",
    )

    direct_joint_time_ms_arg = DeclareLaunchArgument(
        "direct_joint_time_ms",
        default_value="120",
        description="Direct arm serial joint motion time in ms",
    )

    return LaunchDescription([
        joy_dev_arg,
        step_rad_arg,
        direct_joint_time_ms_arg,
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": LaunchConfiguration("joy_dev"),
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }],
        ),
        Node(
            package="joystick_parser",
            executable="joystick_parser",
            name="joystick_parser",
            output="screen",
        ),
        Node(
            package="state_machine",
            executable="arm_joint_servo_test_node",
            name="arm_joint_servo_test_node",
            output="screen",
            parameters=[{
                "servo_count": 5,
                "step_rad": LaunchConfiguration("step_rad"),
                "deadzone": 0.15,
            }],
        ),
        Node(
            package="robot_hardware_interfaces",
            executable="chassis_serial_node",
            name="chassis_serial_node",
            output="screen",
            parameters=[{
                "mode_name": "CHASSIS",
            }],
        ),
        Node(
            package="robot_hardware_interfaces",
            executable="arm_serial_node",
            name="arm_serial_node",
            output="screen",
            parameters=[{
                "mode_name": "ARM",
                "use_moveit_mode": False,
                "direct_joint_time_ms": LaunchConfiguration("direct_joint_time_ms"),
            }],
        ),
    ])
