# pi0grasp
# README: UR5 Robot Control with Pi0 and Orbbec Camera

## Overview

This project integrates a UR5 robotic arm with a vision-language-action (VLA) policy based on the Pi0 model from OpenPI, using sensor data from an Orbbec RGB-D camera. It allows for natural language grasp instructions, which are interpreted by the model to generate joint trajectories.

---

## Project Structure

```
ros2_ws/
├── src/
│   ├── pi0_ur5_grasp/          # Main ROS 2 package for grasping
│   │   ├── launch/             # ROS 2 launch files
│   │   ├── pi0_ur5_grasp/      # Python module with grasp scripts
│   │   │   ├── pi0grasp.py     # Main node
│   │   │   ├── finalgrasp.py   # Final version node
│   │   │   ├── home_pose.py    # Home pose manager
│   │   │   ├── simplegrasp.py  # Simpler policy variant
│   │   │   └── smolvlagrasp.py # SmolVLA baseline (optional)
│   │   ├── resource/           # Package resource marker
│   │   ├── setup.py/setup.cfg/package.xml  # ROS 2 Python packaging
│   └── openpi/                 # Git submodule: OpenPI source code
├── run_pi0grasp.sh             # Shell script to launch the grasp node
├── .env                        # Environment variables (if any)
├── README.md                   # Project documentation
├── ros2_control_reference.md   # ROS 2 usage command reference
└── worklog.md                  # Notes or logs
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

## Contact

Zhang Shuo @ TU Berlin / Fraunhofer

For questions, please reach out via your supervisor or open an issue.
