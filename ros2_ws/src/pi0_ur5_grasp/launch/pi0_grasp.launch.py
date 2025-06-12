from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    orbbec_camera_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'femto_bolt.launch.py'
    )

    # activate Orbbec 
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_launch)
    )

    # activate pi0 grasp 
    pi0_node = Node(
        package='pi0_ur5_grasp',
        executable='pi0_grasp_node',
        name='pi0_grasp_node',
        output='screen',
        parameters=[
            {'prompt': 'pick up the cube'}
        ]
    )

    # activate scaled_joint_trajectory_controller
    activate_scaled_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'set_controller_state',
            'scaled_joint_trajectory_controller', 'active'
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        pi0_node,
        activate_scaled_controller
    ])
