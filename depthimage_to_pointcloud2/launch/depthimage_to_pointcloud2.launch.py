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
        Node(
        package="depthimage_to_pointcloud2",
        executable="depthimage_to_pointcloud2_node",
        output='screen',
        name=[PythonExpression(["'", LaunchConfiguration('full_sensor_topic'), "'.split('/')[-1]"]), '_depth2pc2'],
        parameters=[{'range_max': LaunchConfiguration('range_max'),
                     'use_quiet_nan': LaunchConfiguration('use_quiet_nan')}],
        remappings=[
            ("depth", [LaunchConfiguration('full_sensor_topic'), "/image"]),
            ("depth_camera_info", [LaunchConfiguration('full_sensor_topic'), "/camera_info"]),
            ("pointcloud2", [PythonExpression(["'", LaunchConfiguration('full_sensor_topic'), "'.split('/')[-1]"]), "_pointcloud2"])
            ]
        )
    ])