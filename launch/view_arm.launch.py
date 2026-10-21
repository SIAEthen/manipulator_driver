"""Display the arm URDF using joint feedback from the manipulator driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_dir = get_package_share_directory('seven_function_arm_urdf')
    urdf_file = os.path.join(model_dir, 'urdf', 'seven_function_arm.urdf')
    with open(urdf_file, encoding='utf-8') as model:
        robot_description = model.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_states_topic', default_value='/manipulator_driver/joint_states',
            description='JointState feedback topic published by the driver.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to start RViz.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
            output='screen',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(model_dir, 'rviz', 'view.rviz')],
            output='screen',
        ),
    ])
