from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    filt_node = Node(
            package="pcl_ros",
            # executable="filter_voxel_grid_node",
            executable = "filter_statistical_outlier_removal_node",
            # executable="filter_crop_box_node",
            parameters = [
                {"mean_k": 50},
                {"stddev": 2.0}
                ],
            remappings=[
                ("input", "/filtered_pc"),
                ("output", "filtered_pc_2")
                ]
            )

    return LaunchDescription([
                    filt_node,]
                    )
