import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy


from sensor_msgs.msg import LaserScan

class ScanListener(Node):
    def __init__(self):
        super().__init__('scan_listener')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.get_logger().info("Listening to /scan...")

    def listener_callback(self, msg: LaserScan):
        ranges_msg = "ranges:\n"
        angle_increment_msg = f"angle_increment: {msg.angle_increment}\n"
        for r in msg.ranges:
            ranges_msg += f"- {r}\n"
        self.get_logger().info(angle_increment_msg)
        self.get_logger().info(ranges_msg)

def main(args=None):
    rclpy.init()

    node = ScanListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()