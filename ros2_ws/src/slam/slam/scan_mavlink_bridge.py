#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from pymavlink import mavutil


class RawImuBridge(Node):
    def __init__(self):
        #Create Node
        super().__init__('raw_imu_bridge')

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)

        #Connect the Pixhawk
        self.port = "/dev/ttyACM0"
        self.baud = 115200

        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.get_logger().info(f"Opened serial port {self.port}")

            # Wait for heartbeat
            self.master.wait_heartbeat()
            self.get_logger().info("Heartbeat received")

            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 
                50,  # Hz
                1    # start streaming
            )
            self.get_logger().info("Requested RAW_IMU stream (50 Hz)")

            self.timer = self.create_timer(0.02, self.read_serial)

        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            rclpy.shutdown()

    #Obtain IMU data
    def read_serial(self):
        """Read RAW_IMU messages and publish to ROS topic."""
        msg = self.master.recv_match(type='RAW_IMU', blocking=False)
        if not msg:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = float(msg.xacc)
        imu_msg.linear_acceleration.y = float(msg.yacc)
        imu_msg.linear_acceleration.z = float(msg.zacc)

        imu_msg.angular_velocity.x = float(msg.xgyro)
        imu_msg.angular_velocity.y = float(msg.ygyro)
        imu_msg.angular_velocity.z = float(msg.zgyro)

        # Orientation unknown
        imu_msg.orientation_covariance[0] = -1

        # Publish IMU message
        self.imu_pub.publish(imu_msg)

        self.get_logger().info("Published IMU from RAW_IMU")


def main(args=None):
    rclpy.init(args=args)
    node = RawImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
