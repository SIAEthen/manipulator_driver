import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    left_manipulator_driver=Node(
        package="manipulator_driver",
        executable="driver",
        namespace="left",
        name='manipulator',
        parameters=[{'can_id': 1,
            'joint_names': [
                'joint1', 'joint2', 'joint3', 'joint4',
                'joint5', 'joint6', 'joint_gripper1', 'joint_gripper2'],
            'robot_gripper_torque_direction':1.0,
            'robot_gripper_joint_direction':1.0,
            'can_baud_rate':500,
            
            'robot_joint_offsets': [1.57, -0.12, 2.73, 0.0, 0.0, 0.0],
            
            
            'robot_joint_directions': [-1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
            }],   
        output="screen"
    )
    right_manipulator_driver=Node(
        package="manipulator_driver",
        executable="driver",
        namespace="right",
        name='manipulator',
        parameters=[{'can_id': 0,
            'joint_names': [
                'joint1', 'joint2', 'joint3', 'joint4',
                'joint5', 'joint6', 'joint_gripper1', 'joint_gripper2'],
            'robot_gripper_torque_direction':1.0,
            'robot_gripper_joint_direction':1.0,
            'can_baud_rate':500,
            
            'robot_joint_offsets': [-1.67, -0.07, 3.02, 0.0, 0.0, 0.0],
            
            
            'robot_joint_directions': [1.0, 1.0, -1.0, 1.0, 1.0, 1.0],
            }],   
        output="screen"
    )

    launch_description =LaunchDescription([left_manipulator_driver,right_manipulator_driver])
    return launch_description
