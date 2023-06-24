#!/usr/bin/env python
import os.path as osp

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    # decomp 
    node = Node(
            package="decomp_ros",
            executable="decomp_ros_node",
            name="decomp_ros",
            parameters = [ 
            ],
            remappings = [
                ("cloud_in", "/filtered_pc")
                ],
            )

    return LaunchDescription([
        node,
        ])
            
