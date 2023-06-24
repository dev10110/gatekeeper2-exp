from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    filt_node = Node(
            package="pcl_ros",
            executable="filter_voxel_grid_node",
            # executable="filter_crop_box_node",
            parameters = [
                {"filter_limit_max": 7.0},
                {"leaf_size": 0.05}
                ],
            remappings=[
                ("input", "/depth/color/points"),
                ("output", "filtered_pc")
                ]
            )

    return LaunchDescription([
                    filt_node,]
                    )
