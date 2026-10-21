"""Run gripper calibration against an already running manipulator driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_node', default_value='/manipulator_driver',
            description='Fully qualified driver node name from the arm launch file.'),
        DeclareLaunchArgument(
            'settle_time', default_value='3.0',
            description='Seconds to wait after opening and closing the gripper.'),
        Node(
            package='manipulator_driver',
            executable='gripper_help',
            name='gripper_calibration_helper',
            arguments=[
                '--driver-node', LaunchConfiguration('driver_node'),
                '--settle-time', LaunchConfiguration('settle_time'),
            ],
            output='screen',
        ),
    ])
