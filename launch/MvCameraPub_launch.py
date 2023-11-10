from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikrobot_ros2',
            executable='MvCameraPub',
            name='MvCameraPub'
        )
    ])