# pi0grasp
ros2 command

cd /mnt/HDD/shuo/VLA/ros2_ws
colcon build --symlink-install
source install/setup.bash

source /opt/ros/humble/setup.bash

ros2 launch orbbec_camera femto_bolt.launch.py

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.18.7.1 launch_rviz:=true

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 \
  robot_ip:=172.18.7.1 \
  launch_rviz:=true

ros2 control list_controllers
ros2 control set_controller_state scaled_joint_trajectory_controller activate



---
header:
  stamp:
    sec: 1749044868
    nanosec: 543244451
  frame_id: ''
name:
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
- shoulder_pan_joint
position:
- -1.495906178151266
- 0.18928194046020508
- -0.2825863997088831
- -1.5792220274554651
- 0.03916294872760773
- 1.5943816900253296
velocity:
- 0.0
- -0.0
- 0.0
- 0.0
- 0.0
- 0.0
effort:
- -0.8474043607711792
- -0.573903501033783
- -0.3065332770347595
- 0.030500823631882668
- -0.0045751235447824
- -0.27574270963668823
---

ros2 topic list
/camera/accel/imu_info
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/theora
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/theora
/camera/depth/points
/camera/depth_filter_status
/camera/depth_to_accel
/camera/depth_to_color
/camera/depth_to_gyro
/camera/depth_to_ir
/camera/gyro/imu_info
/camera/gyro_accel/sample
/camera/ir/camera_info
/camera/ir/image_raw
/camera/ir/image_raw/compressed
/camera/ir/image_raw/compressedDepth
/camera/ir/image_raw/theora
/clicked_point
/diagnostics
/dynamic_joint_states
/force_mode_controller/transition_event
/force_torque_sensor_broadcaster/transition_event
/force_torque_sensor_broadcaster/wrench
/forward_position_controller/commands
/forward_position_controller/transition_event
/forward_velocity_controller/commands
/forward_velocity_controller/transition_event
/freedrive_mode_controller/enable_freedrive_mode
/freedrive_mode_controller/transition_event
/goal_pose
/initialpose
/io_and_status_controller/io_states
/io_and_status_controller/robot_mode
/io_and_status_controller/robot_program_running
/io_and_status_controller/safety_mode
/io_and_status_controller/tool_data
/io_and_status_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/joint_trajectory_controller/controller_state
/joint_trajectory_controller/joint_trajectory
/joint_trajectory_controller/state
/joint_trajectory_controller/transition_event
/parameter_events
/passthrough_trajectory_controller/transition_event
/robot_description
/rosout
/scaled_joint_trajectory_controller/controller_state
/scaled_joint_trajectory_controller/joint_trajectory
/scaled_joint_trajectory_controller/state
/scaled_joint_trajectory_controller/transition_event
/speed_scaling_state_broadcaster/speed_scaling
/speed_scaling_state_broadcaster/transition_event
/tcp_pose_broadcaster/pose
/tcp_pose_broadcaster/transition_event
/tf
/tf_static
/ur_configuration_controller/transition_event
/urscript_interface/script_command