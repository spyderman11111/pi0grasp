# pi0grasp
# README: UR5 Robot Control with Pi0 and Orbbec Camera

## Overview

This project integrates a UR5 robotic arm with a vision-language-action (VLA) policy based on the Pi0 model from OpenPI, using sensor data from an Orbbec RGB-D camera. It allows for natural language grasp instructions, which are interpreted by the model to generate joint trajectories.

---

## Project Structure

```
ros2_ws/
├── src/
│   ├── pi0_ur5_grasp/          # Main ROS2 package for grasping
│   ├── openpi/                 # Git submodule: OpenPI source
│   └── ur5_transforms.py       # Input/output transform logic
├── scripts/
│   └── run_pi0grasp.sh         # Script to launch grasp node
├── install/
└── build/
```

---

## Dependencies

* ROS 2 Jazzy
* Python 3.12
* `openpi` repository: cloned and installed locally
* Orbbec Camera SDK + ROS 2 driver
* UR Robot Driver + `scaled_joint_trajectory_controller`
* NVIDIA GPU for inference (optional but recommended)

---

## Build Instructions

```bash
cd /mnt/HDD/shuo/VLA/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Runtime Setup

```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Orbbec camera
ros2 launch orbbec_camera femto_bolt.launch.py

# Launch UR5 driver with RViz
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.18.7.1 launch_rviz:=true

# (Optional) Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 robot_ip:=172.18.7.1 launch_rviz:=true
```

---

## Controller Management

```bash
ros2 control list_controllers
ros2 control set_controller_state scaled_joint_trajectory_controller activate
```

---

## Run Grasp Node

```bash
bash run_pi0grasp.sh
# Enter prompt such as: pick the red cube
```

---

## Input Format (UR5Inputs)

```python
{
  "joints": np.array([...]),
  "base_rgb": np.ndarray [H, W, C],
  "wrist_rgb": np.ndarray [H, W, C] or zeros,
  "prompt": "pick a red cube"
}
```

---

## Output Format (UR5Outputs)

```python
{
  "actions": np.array([...])   # 6-DOF joint positions for UR5
}
```

---

## Example Trajectory Command

```bash
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "
joint_names:
- elbow_joint
- shoulder_lift_joint
- shoulder_pan_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
points:
- positions: [0.0, -1.57, 1.57, 0.0, -1.57, 0.0]
  time_from_start:
    sec: 5"
```

---

## Topic List Reference

Includes:

* `/camera/color/image_raw`
* `/joint_states`
* `/scaled_joint_trajectory_controller/joint_trajectory`
* `/force_torque_sensor_broadcaster/wrench`
* `/io_and_status_controller/robot_mode`

For a full list, run:

```bash
ros2 topic list
```

---

## Contact

Zhang Shuo @ TU Berlin / Fraunhofer

For questions, please reach out via your supervisor or open an issue.
