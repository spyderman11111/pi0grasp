import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import spnav
import time

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

SPACEMOUSE_TO_JOINT_SCALE = {
    'trans_x': 0.002,
    'trans_y': 0.002,
    'trans_z': 0.002,
    'rot_x': 0.01,
    'rot_y': 0.01,
    'rot_z': 0.01
}

class JointControlNode(Node):
    def __init__(self):
        super().__init__('spacemouse_joint_control_node')

        self.current_joint_positions = [0.0] * len(JOINT_NAMES)
        self.joint_state_received = False

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        spnav.spnav_open()
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('Joint control node started.')

    def joint_state_callback(self, msg):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for i, joint in enumerate(JOINT_NAMES):
            if joint in name_to_index:
                self.current_joint_positions[i] = msg.position[name_to_index[joint]]
        self.joint_state_received = True

    def update(self):
        if not self.joint_state_received:
            self.get_logger().warn('Waiting for initial joint state...')
            return

        event = spnav.spnav_poll_event()
        if event and event.ev_type == spnav.SPNAV_EVENT_MOTION:
            dx = event.translation[0] * SPACEMOUSE_TO_JOINT_SCALE['trans_x']
            dy = event.translation[1] * SPACEMOUSE_TO_JOINT_SCALE['trans_y']
            dz = event.translation[2] * SPACEMOUSE_TO_JOINT_SCALE['trans_z']
            rx = event.rotation[0] * SPACEMOUSE_TO_JOINT_SCALE['rot_x']
            ry = event.rotation[1] * SPACEMOUSE_TO_JOINT_SCALE['rot_y']
            rz = event.rotation[2] * SPACEMOUSE_TO_JOINT_SCALE['rot_z']

            deltas = [dx, dy, dz, rx, ry, rz]
            new_positions = [curr + delta for curr, delta in zip(self.current_joint_positions, deltas)]

            traj = JointTrajectory()
            traj.joint_names = JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = new_positions
            point.time_from_start = Duration(sec=1)
            traj.points.append(point)

            self.trajectory_pub.publish(traj)
            self.get_logger().info(f"Published joint delta: {deltas}")

def main():
    rclpy.init()
    node = JointControlNode()
    rclpy.spin(node)
    spnav.spnav_close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
