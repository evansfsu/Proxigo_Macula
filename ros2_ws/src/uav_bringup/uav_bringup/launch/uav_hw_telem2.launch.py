from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="921600"),

        ExecuteProcess(
            cmd=["microxrceagent", "serial", "--dev", dev, "-b", baud],
            output="screen"
        ),

        Node(
            package="uav_guidance",
            executable="offboard_hello",
            name="offboard_hello",
            output="screen",
            parameters=[{
                "use_udp_agent": False
            }],
        ),
    ])
