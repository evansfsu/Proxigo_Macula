from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uav_guidance",
            executable="offboard_hello",
            name="offboard_hello",
            output="screen",
            parameters=[{
                "use_udp_agent": True
            }],
        ),
    ])
