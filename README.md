# depthimage_to_pointcloud2
Modified from https://github.com/ros2/turtlebot2_demo/tree/a612ef2b9b63ba9d19513b67c1d6e5eede680cdf
![colorful_pointcloud2](https://user-images.githubusercontent.com/6606382/164089529-20e523d1-430a-44d0-84f9-be63285d8414.png)


## Usage examples 

### [Command line remapping](https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html):

#### Using the executable directly:
```
ros2 run depthimage_to_pointcloud2 depthimage_to_pointcloud2_node --ros-args -r depth:=/my_robot/my_depth_sensor/image -r depth_camera_info:=/my_robot/my_depth_sensor/camera_info -r pointcloud2:=/my_depth_sensor_pointcloud2 -r __node:=my_new_node_name -p range_max:=19.0 -p use_quiet_nan:=false
```

Using [color](https://wiki.ros.org/rviz/DisplayTypes/PointCloudShared#channels) from a rgb image:
```
ros2 run depthimage_to_pointcloud2 depthimage_to_pointcloud2_node --ros-args -r depth:=/my_robot/my_depth_sensor/image -r depth_camera_info:=/my_robot/my_depth_sensor/camera_info -r pointcloud2:=/my_depth_sensor_pointcloud2 -r __node:=my_new_node_name -p range_max:=19.0 -p use_quiet_nan:=false -p colorful:=true -r image:=/my_robot/my_rgb_image
```
#### Or using the launch file:

```
ros2 launch depthimage_to_pointcloud2 depthimage_to_pointcloud2.launch.py full_sensor_topic:=/my_robot/my_depth_sensor range_max:=19.0 use_quiet_nan:=false rgb_image_topic:=/my_robot/my_rgb_image
```

Using color from a rgb image:
```
ros2 launch depthimage_to_pointcloud2 depthimage_to_pointcloud2.launch.py full_sensor_topic:=/my_robot/my_depth_sensor range_max:=19.0 use_quiet_nan:=false rgb_image_topic:=/my_robot/my_rgb_image
```

__Note:__
* `use_quiet_nan:=true` will show any invalid or out-of-range point as a quiet NaN
* `use_quiet_nan:=false` will show any invalid or out-of-range point as a depth with value range_max (when `range_max!=0.0`).

### [Launch file](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html?highlight=remappings):
#### Simple
```
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(
        package="depthimage_to_pointcloud2",
        executable="depthimage_to_pointcloud2_node",
        output='screen',
        name=`my_node_name`,
        parameters=[{'range_max': '19.0', 
                     'use_quiet_nan': 'false',
                     'colorful': 'true'}],
        remappings=[
            ("image", "/my_rgb_image"),
            ("depth", "/my_depth_sensor/image"),
            ("depth_camera_info", "/my_depth_sensor/camera_info"),
            ("pointcloud2", "/my_output_topic")
            ]
        )
    ])
```
__Note:__
* Just set `'colorful': 'false'` if you don't want to use the color from a rgb image.

#### Complex (using the annoying `PythonExpression`)
```
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'full_sensor_topic',
            default_value=['/my_depth_sensor'],
            description='Base for topic (and node) names'),
        DeclareLaunchArgument(
            'range_max',
            default_value='0.0',
            description='Max range of depth sensor'),
        DeclareLaunchArgument(
            'use_quiet_nan',
            default_value='true',
            description='Use quiet NaN instead of range_max'),
        DeclareLaunchArgument(
            'rgb_image_topic',
            default_value='',
            description='Colorize the point cloud from external RGB image topic'),
        Node(
        package="depthimage_to_pointcloud2",
        executable="depthimage_to_pointcloud2_node",
        output='screen',
        name=[PythonExpression(["'", LaunchConfiguration('full_sensor_topic'), "'.split('/')[-1]"]), '_depth2pc2'],
        parameters=[{'range_max': LaunchConfiguration('range_max'),
                     'use_quiet_nan': LaunchConfiguration('use_quiet_nan'),
                     'colorful': PythonExpression(["'true' if '", LaunchConfiguration('rgb_image_topic'), "' else 'false'"])}],
        remappings=[
            ("image", PythonExpression(["'", LaunchConfiguration('rgb_image_topic'), "' if '", LaunchConfiguration('rgb_image_topic'), "' else 'false'"])),
            ("depth", [LaunchConfiguration('full_sensor_topic'), "/image"]),
            ("depth_camera_info", [LaunchConfiguration('full_sensor_topic'), "/camera_info"]),
            ("pointcloud2", [PythonExpression(["'", LaunchConfiguration('full_sensor_topic'), "'.split('/')[-1]"]), "_pointcloud2"])
            ]
        )
    ])
```

And considering my `full_sensor_topic` is `/my_weird/depth_sensor`:
```
$ ros2 launch depthimage_to_pointcloud2 depthimage_to_pointcloud2.launch.py full_sensor_topic:=/my_weird/depth_sensor
```

It will subscribe to the topics:
```
/my_weird/depth_sensor/image
/my_weird/depth_sensor/camera_info
```

And publish the pointcloud2 on:
```
/depth_sensor_pointcloud2
```
If you want to colorize the pointcloud2 using an external rgb image (with the same dimensions as the depth image), set the parameter `rgb_image_topic` with the rgb image topic:
```
$ ros2 launch depthimage_to_pointcloud2 depthimage_to_pointcloud2.launch.py full_sensor_topic:=/my_weird/depth_sensor rgb_image_topic:=/my_weird/rgb_image
```

Finally, the node name will be `/depth_sensor_pointcloud2` because it gets the `depth_sensor` after splitting (`\`) the `full_sensor_topic` and getting the last item.
