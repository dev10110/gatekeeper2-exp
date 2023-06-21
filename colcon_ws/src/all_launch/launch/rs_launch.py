from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rs_node = Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            parameters=[
                    {"depth_module.profile": "848x480x30"},
                    {"initial_reset": True},
                    {"depth_qos": "SENSOR_DATA"},
                    {"enable_color": True},
                    {"pointcloud.enable": True},
                ]
            )
    off_x = "0.0"
    off_y = "0"
    off_z = "0"
    off_roll = "3.14"
    off_pitch = "0"
    off_yaw = "0"

    static_tf = Node(
            package="tf2_ros", 
            executable="static_transform_publisher",
            arguments=[
               off_x, off_y, off_z, off_yaw, off_pitch, off_roll, "vicon/px4_1/px4_1", "camera_link"]
            )

    return LaunchDescription([
        rs_node,
        static_tf
        ])
