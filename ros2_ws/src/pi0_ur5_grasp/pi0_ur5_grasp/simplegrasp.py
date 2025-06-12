import sys
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from openpi.policies.pi0.modeling_pi0 import PI0Policy
from openpi_client.policies.pi0.modeling_pi0 import PI0Policy
from openpi.models.pi0.modeling_pi0 import Pi0, Pi0Config



class Pi0GraspNode(Node):
    def __init__(self):
        super().__init__('pi0_grasp_node')
        self.prompt = input("Enter grasp instruction: ").strip()
        self.policy = PI0Policy.from_pretrained("lerobot/pi0")

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.latest_image = None
        self.latest_state = None
        self.get_logger().info("Waiting for sensor data...")

    def image_callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        img = img[:, :, ::-1]  # BGR to RGB
        img = np.transpose(img, (2, 0, 1)).astype(np.float32) / 127.5 - 1.0  # Normalize
        self.latest_image = img
        self.try_infer()

    def joint_callback(self, msg):
        if msg.position:
            self.latest_state = np.array(msg.position[:6], dtype=np.float32)  # 取前6维（不含gripper）
            self.try_infer()

    def try_infer(self):
        if self.latest_image is None or self.latest_state is None:
            return

        model_input = {
            "observation.image": self.latest_image,
            "observation.state": self.latest_state,
            "goal": self.prompt,
        }

        try:
            action = self.policy.select_action(model_input)
            action = action[:6] if isinstance(action, np.ndarray) else action["actions"][:6]
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            rclpy.shutdown()
            return

        # 目标顺序：你指定的非标准顺序
        target_order = [
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'shoulder_pan_joint'
        ]

        # π₀默认顺序（用于action index）
        default_order = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        idx_map = {name: i for i, name in enumerate(default_order)}
        ordered_action = [action[idx_map[name]] for name in target_order]

        traj = JointTrajectory()
        traj.joint_names = target_order
        point = JointTrajectoryPoint()
        point.positions = ordered_action
        point.time_from_start = Duration(sec=2)
        traj.points.append(point)

        self.trajectory_pub.publish(traj)
        self.get_logger().info("Action published. Shutting down.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Pi0GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
