# depthimage_to_pointcloud2
Modified from https://github.com/ros2/turtlebot2_demo

## Usage examples 

### [Command line remapping](https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html):
```
ros2 run depthimage_to_pointcloud2 depthimage_to_pointcloud2_node --ros-args -r depth:=/my_depth_sensor/image -r depth_camera_info:=/my_depth_sensor/camera_info -r pointcloud2:=/my_output_topic
```

### [Launch file](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html?highlight=remappings):
```
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
```
