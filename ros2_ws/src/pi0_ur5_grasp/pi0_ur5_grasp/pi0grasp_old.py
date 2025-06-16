import sys
import numpy as np
import dataclasses
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from openpi.training import config as pi0_config
from openpi.policies import policy_config
from openpi.shared import download
from openpi.models import model as _model
from openpi import transforms

def _parse_image(image):
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = np.transpose(image, (1, 2, 0))
    return image.astype(np.uint8)

@dataclasses.dataclass(frozen=True)
class UR5Inputs(transforms.DataTransformFn):
    action_dim: int
    model_type: _model.ModelType = _model.ModelType.PI0

    def __call__(self, data: dict) -> dict:
        joints = data.get("joints", np.zeros(6, dtype=np.float32))
        gripper = data.get("gripper", np.zeros(1, dtype=np.float32))
        state = np.concatenate([joints, gripper])
        state = transforms.pad_to_dim(state, self.action_dim)

        base_image = _parse_image(data["base_rgb"])
        wrist_image = _parse_image(data.get("wrist_rgb", np.zeros_like(base_image)))

        inputs = {
            "observation/image": base_image,
            "observation/wrist_image": wrist_image,
            "observation/state": state,
        }

        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        if "actions" in data:
            inputs["observation/actions"] = transforms.pad_to_dim(data["actions"], self.action_dim)

        return inputs

@dataclasses.dataclass(frozen=True)
class UR5Outputs(transforms.DataTransformFn):
    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"], dtype=np.float32)
        return {"actions": actions[:, :6]}  # 裁剪前6维用于UR5控制

class Pi0GraspNode(Node):
    def __init__(self):
        super().__init__('pi0_grasp_node')
        self.declare_parameter('prompt', '')
        self.input_transform = UR5Inputs(action_dim=32)
        self.output_transform = UR5Outputs()
        prompt_param = self.get_parameter('prompt').get_parameter_value().string_value
        if prompt_param and prompt_param != '':
            self.prompt = prompt_param
        else:
            cmd_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
            self.prompt = ' '.join(cmd_args) if cmd_args else input("Enter a grasp instruction: ").strip()

        self.get_logger().info(f"Using prompt: \"{self.prompt}\"")

        cfg = pi0_config.get_config("pi0_libero")
        ckpt = download.maybe_download("s3://openpi-assets/checkpoints/pi0_libero")
        self.policy = policy_config.create_trained_policy(cfg, ckpt)

        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.home_joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_positions = [0.07505, -1.53058, 1.48675, -0.08071, -1.57239, -0.07874]
        self.init_timer = self.create_timer(1.0, self.send_initial_pose)

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.latest_image = None
        self.latest_joints = None
        self.processed = True

    def send_initial_pose(self):
        traj = JointTrajectory()
        traj.joint_names = self.home_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.velocities = [0.2] * len(self.home_positions)
        point.time_from_start = Duration(sec=5)
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info("Sent home position.")
        self.init_timer.cancel()
        self.processed = False

    def image_callback(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding.lower() == 'bgr8':
            img = img[:, :, ::-1]
        img = np.transpose(img, (2, 0, 1))  # CHW
        self.latest_image = img
        if self.latest_joints and not self.processed:
            self.process_data()

    def joint_callback(self, msg: JointState):
        if msg.position:
            self.latest_joints = {"positions": np.array(msg.position, dtype=np.float32), "names": list(msg.name)}
            if self.latest_image is not None and not self.processed:
                self.process_data()

    def process_data(self):
        self.processed = True
        joints = self.latest_joints["positions"][:6]
        joint_names = self.latest_joints["names"][:6]
        base_img = np.transpose(self.latest_image, (1, 2, 0))

        data = {
            "joints": joints,
            "gripper": np.array([], dtype=np.float32),
            "base_rgb": base_img,
            "wrist_rgb": np.zeros_like(base_img),
            "prompt": self.prompt,
        }

        try:
            model_input = self.input_transform(data)
            result = self.policy.infer(model_input)
            actions = np.asarray(result["actions"])
            self.get_logger().info(f"[DEBUG] Model raw action shape: {actions.shape}")
            self.get_logger().info(f"[DEBUG] First action: {actions[0]}")
            self.get_logger().info(f"[DEBUG] Actions stats (first 50): mean={np.mean(actions):.4f}, std={np.std(actions):.4f}, min={np.min(actions):.4f}, max={np.max(actions):.4f}")
            last_dim = actions[:, 6]
            self.get_logger().info(f"[DEBUG] 7th dim stats (possibly gripper?): mean={np.mean(last_dim):.4f}, std={np.std(last_dim):.4f}, min={np.min(last_dim):.4f}, max={np.max(last_dim):.4f}")

            if np.isnan(actions).any():
                self.get_logger().warn("[WARNING] NaNs detected in action output.")

            output = self.output_transform({"actions": actions})
            self.action_sequence = output["actions"]  # shape (50, 6)
            self.joint_names = joint_names
            self.current_step = 0
            self.action_timer = self.create_timer(2.0, self.execute_next_step)

        except Exception as e:
            self.get_logger().error(f"[ERROR] Inference failed: {e}")
            rclpy.shutdown()

    def execute_next_step(self):
        if self.current_step >= len(self.action_sequence):
            self.action_timer.cancel()
            self.get_logger().info("All steps done. Returning to home.")
            self.return_timer = self.create_timer(2.0, self.return_home)
            return

        joint_targets = self.action_sequence[self.current_step]
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_targets.tolist()
        point.time_from_start = Duration(sec=2)
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info(f"[Step {self.current_step+1}] Joint targets: {joint_targets.tolist()}")
        self.current_step += 1

    def return_home(self):
        self.return_timer.cancel()
        traj = JointTrajectory()
        traj.joint_names = self.home_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start = Duration(sec=2)
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info("Returned to home position.")
        new_prompt = input("Enter new grasp instruction (or 'exit'): ").strip()
        if new_prompt.lower() in ["", "exit", "quit"]:
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.prompt = new_prompt
            self.processed = False

def main(args=None):
    rclpy.init(args=args)
    node = Pi0GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
