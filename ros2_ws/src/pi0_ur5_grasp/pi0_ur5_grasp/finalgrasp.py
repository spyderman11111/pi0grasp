import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from lerobot.common.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.configs.types import FeatureType, PolicyFeature

# --- Define UR5Inputs and UR5Outputs classes (unchanged) ---
class UR5Inputs:
    """Convert UR5 sensor data (images + joint state) to π0 model input format."""
    def __init__(self, action_dim=14, adapt_to_pi=True):
        """
        action_dim: expected action dimension of the model (π0 uses 14 for arms):contentReference[oaicite:23]{index=23}.
        adapt_to_pi: whether to transform joints to π0 internal frame.
        """
        self.action_dim = action_dim
        self.adapt_to_pi = adapt_to_pi
        # Expected camera channels for UR5 (base RGB and wrist RGB, for example)
        self.EXPECTED_CAMERAS = ("base_rgb", "wrist_rgb")
    
    def __call__(self, data: dict) -> dict:
        """
        data: expects {'images': {name: np.array}, 'state': np.array} with 
              joint angles in radians and images as numpy arrays.
        Returns a dict ready for the π0 policy (keys: 'observation.image', 'observation.state').
        """
        # 1. Adapt joint state to π0 internal representation if needed.
        state = data["state"].astype(np.float32)
        if self.adapt_to_pi:
            # (Placeholder for any specific joint angle flips or reordering needed)
            # For simplicity, assume no reordering needed for UR5, or adapt handled internally.
            # In real implementation, apply flip mask or transforms if provided.
            pass
        # 2. Pad the state to action_dim length (14):contentReference[oaicite:24]{index=24}.
        if state.shape[0] < self.action_dim:
            pad_len = self.action_dim - state.shape[0]
            state = np.concatenate([state, np.zeros(pad_len, dtype=np.float32)])
        else:
            state = state[:self.action_dim]
        # 3. Prepare image inputs.
        images = data.get("images", {})
        processed_images = {}
        for cam in self.EXPECTED_CAMERAS:
            if cam in images:
                img = images[cam].astype(np.float32)
                # Normalize image to [-1,1] if needed (π0 expects normalized images).
                # Here, convert uint8 [0,255] to float32 [-1.0, 1.0].
                if img.dtype == np.uint8:
                    img = img / 127.5 - 1.0
                processed_images[cam] = img
            else:
                # Missing camera: use a black image (zeros):contentReference[oaicite:25]{index=25}.
                # Determine expected shape from any available image or default (3x224x224).
                if processed_images:
                    # use shape of an existing image
                    ref_img = next(iter(processed_images.values()))
                    zero_img = np.zeros_like(ref_img, dtype=np.float32)
                else:
                    # default to 3x224x224
                    zero_img = np.zeros((3, 224, 224), dtype=np.float32)
                processed_images[cam] = zero_img
        # At this point, processed_images has at least 'base_rgb' and 'wrist_rgb'.
        # We'll use only 'base_rgb' and possibly 'wrist_rgb' if provided.
        # 4. Construct model input dict.
        # Use the first available camera as "observation.image" (assuming base_rgb).
        # If multiple images were expected, we could supply them as separate keys (e.g., observation.images.*).
        # For simplicity, we feed the base camera as observation.image.
        obs = {}
        if "base_rgb" in processed_images:
            # The model config uses 'observation.image' as key (single image):contentReference[oaicite:26]{index=26}.
            obs["observation.image"] = processed_images["base_rgb"]
        else:
            # Fallback: if base_rgb not present, but we ensured it is, so not expected here.
            obs["observation.image"] = next(iter(processed_images.values()))
        obs["observation.state"] = state
        return obs

class UR5Outputs:
    """Convert π0 model output to UR5 joint target angles."""
    def __init__(self, adapt_to_pi=True):
        """
        adapt_to_pi: whether model output is in π0 internal space and needs transformation back.
        """
        self.adapt_to_pi = adapt_to_pi
    
    def __call__(self, data):
        """
        data: model output, can be a dict with 'action(s)' or a numpy array of actions.
        Returns a dict with 'actions': target joint angles for UR5.
        """
        # Normalize input format
        if isinstance(data, dict) and "actions" in data:
            actions = np.asarray(data["actions"])
        else:
            actions = np.asarray(data)  # assume data itself is the action array
        # If the policy outputs a sequence, take the first step
        if actions.ndim > 1:
            # e.g., shape (T, 14), take index 0
            actions = actions[0]
        # Ensure it's 1D now of length 14.
        actions = actions.flatten()
        # If adapt was applied on input, we may need to invert that transform here.
        if self.adapt_to_pi:
            # (Placeholder: apply inverse of any joint flips or normalization if needed)
            pass
        # Take the first 7 values as UR5 joint targets (6 arm joints + gripper):contentReference[oaicite:27]{index=27}.
        target_joints = actions[:7]
        return {"actions": target_joints}

# --- Define the ROS2 Node class with combined logic ---
class Pi0GraspNode(Node):
    def __init__(self):
        super().__init__('pi0_grasp_node')
        # Declare and/or get the prompt parameter
        self.declare_parameter('prompt', '')
        prompt_param = self.get_parameter('prompt').get_parameter_value().string_value
        # Determine prompt source
        if prompt_param and prompt_param != '':
            self.prompt = prompt_param
        else:
            # If no ROS param, check command-line args (excluding ROS args)
            cmd_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
            if cmd_args:
                # Join all cmd args as one prompt string
                self.prompt = ' '.join(cmd_args)
            else:
                # If no args, ask user (for direct run convenience)
                self.prompt = input("Enter a grasp instruction for the robot: ").strip()
        self.get_logger().info(f"Using prompt: \"{self.prompt}\"")
        
        # Load the π0 policy model:contentReference[oaicite:28]{index=28}.
        self.get_logger().info("Loading π0 model (this may take a few seconds)...")
        self.policy = PI0Policy.from_pretrained("lerobot/pi0")
        # Configure model input/output features:contentReference[oaicite:29]{index=29}:contentReference[oaicite:30]{index=30}.
        self.policy.config.output_features = {
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(14,))
        }
        self.policy.config.input_features = {
            "observation.image": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 224, 224)),
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(14,))
        }
        # Initialize UR5 input/output transformers
        self.ur5_inputs = UR5Inputs(action_dim=14, adapt_to_pi=True)
        self.ur5_outputs = UR5Outputs(adapt_to_pi=True)
        # Publisher for joint trajectory
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        # Define home position (6 joints for UR5 arm)
        self.home_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_positions = [-1.495906178151266, 0.18928194046020508, -0.2825863997088831,
                                -1.5792220274554651, 0.03916294872760773, 1.5943816900253296]
        # Send robot to home at startup (after 1s)
        self.init_timer = self.create_timer(1.0, self.send_initial_pose)
        # Subscribe to sensor topics
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        # Storage for received messages
        self.latest_image = None
        self.latest_joints = None
        # Flag to control processing
        # Start as True to prevent immediate processing until initial home pose is sent
        self.processed = True
        self.get_logger().info("Pi0GraspNode initialized, waiting for sensor data...")

    def send_initial_pose(self):
        """Publish a trajectory to move the robot to the home position, then allow processing."""
        traj = JointTrajectory()
        traj.joint_names = self.home_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start.sec = 2  # 2 seconds to reach home position
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info("Initial home position trajectory published.")
        # Cancel the init timer (one-shot behavior)
        self.init_timer.cancel()
        # Allow processing sensor data after initial home command
        self.processed = False

    def image_callback(self, msg: Image):
        # Convert ROS Image message to numpy array
        height, width = msg.height, msg.width
        img_format = msg.encoding  # e.g., "rgb8" or "bgr8"
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        # If the image data includes padding (step != width * channels), handle row stride
        if msg.step != width * len(msg.data) / height:
            img_data = img_data.reshape((height, msg.step))
            img_data = img_data[:, :width * (msg.step // width)]  # remove padding
            img_data = img_data.reshape(height, width, -1)
        else:
            img_data = img_data.reshape((height, width, -1))
        # Convert to RGB if needed
        if img_format.lower() == 'bgr8':
            img_data = img_data[:, :, ::-1]
        # Transpose to channel-first format (3 x H x W)
        img_data = np.transpose(img_data, (2, 0, 1))
        self.latest_image = img_data.astype(np.uint8)  # store as uint8 or float, UR5Inputs will handle normalization
        # If we have joint data and not processed yet, trigger processing
        if self.latest_joints is not None and not self.processed:
            self.process_data()

    def joint_callback(self, msg: JointState):
        # Extract joint positions as numpy array
        if msg.position:
            joint_positions = np.array(msg.position, dtype=np.float32)
        else:
            # If no positions in message, skip
            return
        self.latest_joints = {
            "positions": joint_positions,
            "names": list(msg.name)
        }
        # If we have an image and not processed yet, trigger processing
        if self.latest_image is not None and not self.processed:
            self.process_data()

    def process_data(self):
        """Once both image and joint data are available, run model inference and publish trajectory."""
        # Mark as processed to avoid re-entry until reset
        self.processed = True
        # Prepare the model input using UR5Inputs
        images_dict = {"base_rgb": self.latest_image}
        state_vec = self.latest_joints["positions"]
        ur5_data = {"images": images_dict, "state": state_vec}
        model_input = self.ur5_inputs(ur5_data)  # standardized input for π0 model
        # Include the language prompt in the model input (assuming key 'goal' is used by policy)
        model_input["goal"] = self.prompt
        # Run model inference to get action
        try:
            action = self.policy.select_action(model_input)
        except Exception as e:
            self.get_logger().error(f"Model inference failed: {e}")
            # Shutdown if inference fails
            self.destroy_node()
            rclpy.shutdown()
            return
        # Convert model output to target joint angles
        target = self.ur5_outputs({"actions": action})
        target_joints = target["actions"]  # numpy array of 7 target joint positions (6 arm joints + gripper)
        self.get_logger().info(f"Predicted joint targets: {target_joints.tolist()}")
        # Build JointTrajectory message for the action
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()  # current time
        # Use the same joint name order as received in JointState for consistency
        traj_msg.joint_names = self.latest_joints["names"]
        point = JointTrajectoryPoint()
        point.positions = target_joints.astype(float).tolist()
        point.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds to reach target
        traj_msg.points.append(point)
        # Publish the trajectory for the action
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info("Published JointTrajectory to controller.")
        # Schedule the robot's return to home after executing the action
        self.get_logger().info("Action published, scheduling return to home...")
        # Create a one-shot timer to return home after 3 seconds (allow action to execute)
        self.return_timer = self.create_timer(3.0, self.return_home)

    def return_home(self):
        """Publish a trajectory to return the robot to home, then prompt for next instruction or shut down."""
        # Cancel this timer to make it one-shot
        self.return_timer.cancel()
        # Publish trajectory to home position
        traj = JointTrajectory()
        traj.joint_names = self.home_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start.sec = 2  # 2 seconds to reach home position
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info("Returning to home position trajectory published.")
        # Prompt the user for the next instruction
        new_prompt = input("Enter a new grasp instruction for the robot (or 'exit' to quit): ").strip()
        if new_prompt == '' or new_prompt.lower() in ['exit', 'quit']:
            # If no prompt or an exit command is given, shut down the node
            self.get_logger().info("No new prompt (or exit command received). Shutting down node.")
            self.destroy_node()
            rclpy.shutdown()
        else:
            # Update the prompt and allow processing of the next action
            self.prompt = new_prompt
            self.get_logger().info(f"Using prompt: \"{self.prompt}\"")
            # Reset processed flag to process the next incoming sensor data
            self.processed = False

def main(args=None):
    rclpy.init(args=args)
    node = Pi0GraspNode()
    try:
        # Spin the node so callbacks are processed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        # Ensure proper shutdown
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
