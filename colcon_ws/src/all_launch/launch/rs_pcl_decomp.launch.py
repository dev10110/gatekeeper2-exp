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

    rs_pcl = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                osp.join( get_package_share_directory("all_launch"), "launch"),
                "/rs_pcl.launch.py"])
            )

    decomp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                osp.join( get_package_share_directory("all_launch"), "launch"),
                "/decomp.launch.py"])
            )

    return LaunchDescription([
        rs_pcl, 
        decomp
        ])
