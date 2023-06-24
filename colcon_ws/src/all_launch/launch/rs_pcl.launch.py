
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    rs_node = ComposableNode(
            package="realsense2_camera",
            plugin="realsense2_camera::RealSenseNodeFactory",
            parameters=[
                    # {"depth_module.profile": "848x480x30"},
                    {"depth_module.profile": "640x480x30"},
                    {"initial_reset": True},
                    {"depth_qos": "SENSOR_DATA"},
                    {"enable_color": True},
                    {"pointcloud.enable": True},
                    {"enable_infra1": False},
                    {"enable_infra2": False},
                ],
            extra_arguments=[{'use_intraprocess_comms': True}],
            )

    filt_node = ComposableNode(
            package="pcl_ros",
            plugin="pcl_ros::VoxelGrid",
            # executable="filter_crop_box_node",
            parameters = [
                {"filter_limit_min": 0.1},
                {"filter_limit_max": 7.0},
                {"leaf_size": 0.025}
                ],
            remappings=[
                ("input", "/depth/color/points"),
                ("output", "filtered_pc")
                ],
            extra_arguments=[{'use_intraprocess_comms': True}],
            )

    return LaunchDescription([

        ComposableNodeContainer(
            name="filter_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions = [
                rs_node, 
                filt_node
                ],
            output="screen"
            )
        ]
        )













