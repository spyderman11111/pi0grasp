import os
from pathlib import Path
import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get package share directories
    pkg_root = get_package_share_directory("ipk_moveit")
    ur_description_share_dir = get_package_share_directory('ur_description')
    ur_moveit_config_share_dir = get_package_share_directory('ur_moveit_config')
    
    # Define the yaml configuration file
    yaml_file = Path(pkg_root) / "config" / "ur_moveit_config.yaml"
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur")
        .robot_description(
            file_path=Path(ur_description_share_dir) / "urdf" / "ur.urdf.xacro",
            mappings={"name": "ur10", "ur_type": "ur10"}
        )
        .robot_description_semantic(
            file_path=Path(ur_moveit_config_share_dir) / "srdf" / "ur.srdf.xacro",
            mappings={"name": "ur10", "ur_type": "ur10"}
        )
        .robot_description_kinematics()
        .moveit_cpp(file_path=yaml_file)
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(Path(pkg_root) / "config" / "ur_twist_config.yaml")
        .to_dict()
    }

    # Additional parameters
    #acceleration_filter_update_period = {"update_period": 0.01}
    #planning_group_name = {"planning_group_name": "ur_manipulator"}

    # Launch Servo node
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            #acceleration_filter_update_period,
            #planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # Include UR control launch file
    ur_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": "192.168.29.102",
            "launch_rviz": "False",
            "initial_joint_controller":"joint_trajectory_controller",
            "use_mock_hardware": "False"
        }.items()
    )
    
    return launch.LaunchDescription([
        ur_node,
        servo_node
    ])
