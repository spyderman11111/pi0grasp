from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spacemouse_joint_control',
            executable='joint_controller_node',
            name='spacemouse_joint_controller',
            output='screen',
        )
    ])
