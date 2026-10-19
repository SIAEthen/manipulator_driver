import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    manipulator_driver=Node(
        package="manipulator_driver",
        executable="driver",
        namespace="left",
        name='manipulator',
        parameters=[{'can_id_': 1,
            'can_baud_rate_':500}],   
        output="screen"
    )

    launch_description =LaunchDescription()
    launch_description.add_action(manipulator_driver)
    return launch_description
