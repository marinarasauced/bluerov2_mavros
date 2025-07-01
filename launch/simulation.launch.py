from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosmav",
            executable="ros_bluerov2_interface",
            name="bluerov2_interface",
            namespace="rov1",
            parameters=[{
                "udp_params": "udpin:0.0.0.0:14550"
            }]
        ),
        Node(
            package="rosmav",
            executable="ros_bluerov2_interface",
            name="bluerov2_interface",
            namespace="rov2",
            parameters=[{
                "udp_params": "udpin:0.0.0.0:14560"
            }]
        )
    ])
