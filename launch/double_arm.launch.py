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
        parameters=[{'can_id_': 1,
            'can_baud_rate_':500,
            
            'fauvms_joint1_offset_':1.57,
            'fauvms_joint2_offset_':-0.12,
            'fauvms_joint3_offset_':2.73,
            'fauvms_joint4_offset_':0.0,
            'fauvms_joint5_offset_':0.0,
            'fauvms_joint6_offset_':0.0 ,
            
            'fauvms_joint1_direction_':-1.0,
            'fauvms_joint2_direction_':1.0,
            'fauvms_joint3_direction_':-1.0,
            'fauvms_joint4_direction_':1.0,
            'fauvms_joint5_direction_':1.0,
            'fauvms_joint6_direction_':1.0            }],   
        output="screen"
    )
    right_manipulator_driver=Node(
        package="manipulator_driver",
        executable="driver",
        namespace="right",
        name='manipulator',
        parameters=[{'can_id_': 0,
            'can_baud_rate_':500,
            
            'fauvms_joint1_offset_':-1.67,
            'fauvms_joint2_offset_':-0.07,
            'fauvms_joint3_offset_':3.02,
            'fauvms_joint4_offset_':0.0,
            'fauvms_joint5_offset_':0.0,
            'fauvms_joint6_offset_':0.0,
            
            'fauvms_joint1_direction_':1.0,
            'fauvms_joint2_direction_':1.0,
            'fauvms_joint3_direction_':-1.0,
            'fauvms_joint4_direction_':1.0,
            'fauvms_joint5_direction_':1.0,
            'fauvms_joint6_direction_':1.0              }],   
        output="screen"
    )

    launch_description =LaunchDescription([left_manipulator_driver,right_manipulator_driver])
    return launch_description
