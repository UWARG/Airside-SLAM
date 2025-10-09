#using mavros to get mavlink data
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMavlinkBridge(Node):
    def __init__(self):
        super().__init__('odom_mavlink_bridge')
        self.subscriber = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/pixhawk/odom', 10)
        
    def odom_callback(self, msg: Odometry):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomMavlinkBridge()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()