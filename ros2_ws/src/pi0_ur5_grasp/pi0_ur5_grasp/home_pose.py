import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR5InitPose(Node):
    def __init__(self):
        super().__init__('ur5_init_pose_sender')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.send_trajectory)

    def send_trajectory(self):
        joint_names = [
            'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint'
        ]

        home_positions = [
            -1.4959, 0.1893, -0.2826,
            -1.5792, 0.0392, 1.5944
        ]

        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = home_positions
        point.velocities = [0.3] * len(home_positions)  
        point.time_from_start.sec = 3  

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info("Initial pose with velocity published.")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = UR5InitPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
