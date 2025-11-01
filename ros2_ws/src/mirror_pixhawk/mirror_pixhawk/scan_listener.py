import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

from sensor_msgs.msg import LaserScan
from pymavlink import mavutil 


CONNECTION_STRING = "/dev/ttyACM0"
DELETE_INDEX = set([round(98.0/72*i) for i in range(1,29)])

class ScanListener(Node):
    def __init__(self):
        super().__init__('scan_listener')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.connection = mavutil.mavlink_connection(CONNECTION_STRING)
        self.get_logger().info(f'Connecting to Pixhawk at {CONNECTION_STRING}...')
        # self.connection.wait_heartbeat()
        # self.get_logger().info('Pixhawk heartbeat received.')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info("Listening to /scan...")
       

    def listener_callback(self, msg: LaserScan):

        ################## Print out info subscribed to LIDAR
        ranges_msg = "ranges:\n"
        for r in msg.ranges:
            ranges_msg += f"- {r}\n"
        angle_increment_msg = f"angle_increment: {msg.angle_increment}\n"
        num_ranges_msg = f"Number of measurements: {len(msg.ranges)}\n"
        

        self.get_logger().info(f"Header: \nSeconds: {msg.header.stamp.sec}\nNanoseconds: {msg.header.stamp.nanosec}\nFrame ID: {msg.header.frame_id}\n")
        self.get_logger().info(f"Min angle: {msg.angle_min}\n")
        # self.get_logger().info()
        self.get_logger().info(angle_increment_msg)
        self.get_logger().info(num_ranges_msg)
        self.get_logger().info(f"Time increment: {msg.time_increment}\n")
        self.get_logger().info(f"Scan Time: {msg.scan_time}\n")
        self.get_logger().info(f"Range min: {msg.range_min}\n")
        self.get_logger().info(f"Range max: {msg.range_max}\n")
        self.get_logger().info(ranges_msg)

        adjusted_ranges = []
        # # LaserScan ranges measures 100 ranges, but MavLink obstacle_distance only takes an array of 72 elements
        # msg.ranges = [x for x in msg.ranges if x > msg.range_min and x < msg.range_max]
        # if len(msg.ranges) == 72:
        #     adjusted_ranges = msg.ranges
        # elif len(msg.ranges) < 72:


        # # delete_index = set()
        # for i,v in enumerate(msg.ranges):
        #     if i not in DELETE_INDEX:
        #         if v == float('inf'):
        #             adjusted_ranges.append(round(msg.range_max*100)+1) # max + 1 to model inf
        #         else:
        #             adjusted_ranges.append(round(v*100))
        # # adjusted_ranges = [round(v*100) for i,v in enumerate(msg.ranges) if i not in DELETE_INDEX and v != inf]

        # ranges_msg = ""
        # for r in adjusted_ranges:
        #     ranges_msg += f"- {r}\n"
        # self.get_logger().info(ranges_msg)

############# SEND MESSAGE ########
        # self.connection.mav.obstacle_distance_send(
        #     time_usec=msg.header.stamp.sec,
        #     sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        #     distances=adjusted_ranges, # LaserScan ranges[]
        #     increment=round(msg.angle_increment*180/math.pi), # LaserScan angle_increment is in rad, MavLink increment is in degrees
        #     min_distance=round(msg.range_min*100), # LaserScan range_min is in meters, MavLink min_distance is in cm
        #     max_distance=round(msg.range_max*100),
        # )
        # self.get_logger().info("Message sent successfully")



def main():
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
