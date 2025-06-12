# ROS 2 Command Reference (UR5 + Orbbec + π0)

Author: Zhang Shuo  
Last Updated: 2025-06-12  
System: ROS 2 Jazzy (or Humble), Python 3.12, UR5 + Orbbec Femto Bolt

---

## 📦 Build the Workspace

```bash
cd /mnt/HDD/shuo/VLA/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Tip: Always run `source /opt/ros/humble/setup.bash` first to avoid missing dependencies.

---

## 🎥 Launch the Orbbec Camera

```bash
ros2 launch orbbec_camera femto_bolt.launch.py
```

---

## 🤖 Launch the UR5 Robot Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=172.18.7.1 \
  launch_rviz:=true
```

Optional MoveIt interface:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 \
  robot_ip:=172.18.7.1 \
  launch_rviz:=true
```

---

## 🧠 π0 Grasp Node Example

Run the policy-based control node:

```bash
bash run_pi0grasp.sh
```

You will be prompted to input a grasp command, such as:

```
pick the red cube
```

---

## 🔧 Controller Management

### List all controllers:

```bash
ros2 control list_controllers
```

### Activate the trajectory controller (if needed):

```bash
ros2 control set_controller_state scaled_joint_trajectory_controller activate
```

---

## 📡 ROS 2 Topics Overview

Use `ros2 topic list` to see all available topics. Example output:

```
/joint_states  
/scaled_joint_trajectory_controller/joint_trajectory  
/camera/color/image_raw  
...
```

---

## 🧪 Joint State Format Example

```yaml
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
  - -1.4959
  - 0.1892
  - -0.2825
  - -1.5792
  - 0.0391
  - 1.5943
```

---

## 📨 Manually Send a Trajectory

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

To continuously publish at a fixed rate, use a Python node or `ros2 topic pub` in a loop.

---

## 🧩 Notes

- Ensure all joint names in the published trajectory match your robot's controller.