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

    # octomap
    node = Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            parameters = [ 
                {"resolution": 0.05},
                {"frame_id": "vicon/world"},
                {"sensor_model.max_range": 4.0},
                {"colored_map": False},
                {"filter_ground_plane", "false"},
                {"occupancy_max_z": 2.5},
                {"occupancy_min_z": -0.1},
                {"publish_free_space": False},
            ],
            remappings = [
                ("cloud_in", "/depth/color/points")
                ],
            #arguments = [
            #    '--ros-args', '--log-level', 'debug']
            )

    # octomap_launch = IncludeLaunchDescription(
    #         XMLLaunchDescriptionSource([
    #             osp.join( get_package_share_directory('octomap_server'), 'launch'),
    #             '/octomap_mapping.launch.xml']),
    #             )

    return LaunchDescription([
        node,
        ])
            
