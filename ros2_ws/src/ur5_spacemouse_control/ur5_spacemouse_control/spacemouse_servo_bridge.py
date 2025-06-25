import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import ctypes

# 加载 libspnav 动态库
libspnav = ctypes.cdll.LoadLibrary("libspnav.so")

# 定义 SpaceMouse 事件结构体
class SpnavEvent(ctypes.Structure):
    _fields_ = [
        ("ev_type", ctypes.c_int),
        ("translation", ctypes.c_int * 3),
        ("rotation", ctypes.c_int * 3)
    ]

SPNAV_EVENT_MOTION = 1
SCALE_TRANSLATION = 0.0005  # 缩放因子：位置（单位转换）
SCALE_ROTATION = 0.001      # 缩放因子：旋转（单位转换）

class SpacemouseToServoBridge(Node):
    def __init__(self):
        super().__init__('spacemouse_to_servo_bridge')

        if libspnav.spnav_open() == -1:
            self.get_logger().error("无法打开 spacenav 设备（libspnav）")
            exit(1)

        self.publisher = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.05, self.publish_twist)  # 20 Hz
        self.get_logger().info("Spacemouse Servo Bridge 节点已启动。")

    def publish_twist(self):
        event = SpnavEvent()
        if libspnav.spnav_poll_event(ctypes.byref(event)) > 0 and event.ev_type == SPNAV_EVENT_MOTION:
            dx = event.translation[0] * SCALE_TRANSLATION
            dy = event.translation[1] * SCALE_TRANSLATION
            dz = event.translation[2] * SCALE_TRANSLATION
            rx = event.rotation[0] * SCALE_ROTATION
            ry = event.rotation[1] * SCALE_ROTATION
            rz = event.rotation[2] * SCALE_ROTATION

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"  # 必须与 Servo 配置一致
            twist.twist.linear.x = dx
            twist.twist.linear.y = dy
            twist.twist.linear.z = dz
            twist.twist.angular.x = rx
            twist.twist.angular.y = ry
            twist.twist.angular.z = rz

            self.publisher.publish(twist)
            self.get_logger().info(
                f"[TWIST] lin: [{dx:.4f}, {dy:.4f}, {dz:.4f}], ang: [{rx:.4f}, {ry:.4f}, {rz:.4f}]"
            )


    def destroy_node(self):
        libspnav.spnav_close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SpacemouseToServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
