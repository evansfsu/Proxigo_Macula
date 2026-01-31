from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='uav_guidance',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        emulate_tty=True,
    ))

    return ld
