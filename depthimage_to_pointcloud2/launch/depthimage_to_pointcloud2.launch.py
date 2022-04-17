from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(
        package="depthimage_to_pointcloud2",
        executable="depthimage_to_pointcloud2_node",
        remappings=[
            ("depth", "/my_depth_sensor/image"),
            ("depth_camera_info", "/my_depth_sensor/camera_info"),
            ("pointcloud2", "/my_output_topic")
            ]
        )
    ])