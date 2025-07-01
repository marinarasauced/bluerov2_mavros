from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rosmav'),
        'config',
        'bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[{'config_file': config_path}]
        ),
        Node(
            package='rosmav',
            executable='bluerov2_simulation_interface',
            name='bluerov2_simulation_interface',
            output='screen',
            namespace='rov1'
        ),
        Node(
            package='rosmav',
            executable='bluerov2_simulation_interface',
            name='bluerov2_simulation_interface',
            output='screen',
            namespace='rov2'
        ),
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package="rosmav",
#             executable="ros_bluerov2_interface",
#             name="bluerov2_interface",
#             namespace="rov1",
#             parameters=[{
#                 "udp_params": "udpin:0.0.0.0:14550"
#             }]
#         ),
#         Node(
#             package="rosmav",
#             executable="ros_bluerov2_interface",
#             name="bluerov2_interface",
#             namespace="rov2",
#             parameters=[{
#                 "udp_params": "udpin:0.0.0.0:14560"
#             }]
#         )
#     ])
