from pymavlink import mavutil
import time

class DroneConnection:
    __create_key = object()

    def __init__(self, drone):
        self.drone = drone
        self.time = time.monotonic()

    @classmethod
    def create(cls, address:str = "/dev/ttyAMA0", baud: int = 57600) -> "tuple[bool, DroneConnection | None]":
        drone = mavutil.mavlink_connection(address, baud=baud, source_component=191, source_system=1)
        drone.wait_heartbeat()
        print("Heartbeat from drone successfully received")

        return True, DroneConnection(drone)

    def send_odometry(self, x, y, z, q, vx, vy, vz, angular_vx, angular_vy, angular_vz):
        """
        Sends an ODOMETRY MAVLink message.

        :param master: Pymavlink connection object.
        :param time_usec: Timestamp (microseconds, since boot or Unix epoch).
        :param x, y, z: Position in meters (e.g., NED frame).
        :param q: Quaternion for orientation [w, x, y, z].
        :param vx, vy, vz: Linear velocity in m/s.
        :param angular_vx, angular_vy, angular_vz: Angular velocity in rad/s.
        """
        frame_id = mavutil.mavlink.MAV_FRAME_LOCAL_FRD
        child_frame_id = mavutil.mavlink.MAV_FRAME_BODY_FRD # often used for velocity estimates

        time_usec = int((time.monotonic() - self.time) * 1e6)

        self.drone.mav.odometry_send(
            time_usec,          # time_usec (uint64_t)
            frame_id,           # frame_id (uint8_t)
            child_frame_id,     # child_frame_id (uint8_t)
            x, y, z,            # x, y, z (float, meters)
            q,                  # q (float[4], quaternion w, x, y, z)
            vx, vy, vz,         # vx, vy, vz (float, m/s)
            angular_vx,         # angular_vx (float, rad/s)
            angular_vy,         # angular_vy (float, rad/s)
            angular_vz,         # angular_vz (float, rad/s)
            # Start of pose covariance matrix
            [0.01,  0.0,   0.0,   0.0,   0.0,   0.0] + \
                   [0.01,  0.0,   0.0,   0.0,   0.0] + \
                          [0.01,  0.0,   0.0,   0.0] + \
                                 [0.005, 0.0,   0.0] + \
                                        [0.005, 0.0] + \
                                              [0.01],
            # End of pose covariance matrix
            # Start of velocity covariance matrix
            [0.04,  0.0,   0.0,   0.0,   0.0,   0.0] + \
                   [0.04,  0.0,   0.0,   0.0,   0.0] + \
                          [0.04,  0.0,   0.0,   0.0] + \
                                  [0.02, 0.0,   0.0] + \
                                         [0.02, 0.0] + \
                                              [0.05],
            # End of velocity covariance matrix
            0,                  # reset_counter (uint8_t, 0 for no reset)
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO # estimator type
        )

    def send_vision_position_estimate(self, x, y, z, roll, pitch, yaw):
        """
        Sends an VISION_POSITION_ESTIMATE MAVLink message.

        :param x, y, z: Position in meters
        :param roll, pitch, yaw: angles in radians
        """
        time_usec = int(time.time() * 1e6)

        self.drone.mav.vision_position_estimate_send(
            usec=time_usec,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            covariance=[0.0001,  0.0,   0.0,   0.0,   0.0,   0.0] + \
                              [0.0001,  0.0,   0.0,   0.0,   0.0] + \
                                     [0.0001,  0.0,   0.0,   0.0] + \
                                            [0.00005, 0.0,   0.0] + \
                                                   [0.00005, 0.0] + \
                                                         [0.0001],
        )

    def send_vision_speed_estimate(self, x, y, z):
        """
        Sends an VISION_SPEED_ESTIMATE MAVLink message

        :param x, y, z: speeds in meters/seconds 
        """
        time_usec = int(time.time() * 1e6)

        self.drone.mav.vision_speed_estimate_send(
            usec=time_usec,
            x=x,
            y=y,
            z=z,
            covariance= [0.0004,  0.0,   0.0,   0.0,   0.0,   0.0] + \
                               [0.0004,  0.0,   0.0,   0.0,   0.0] + \
                                      [0.0004,  0.0,   0.0,   0.0] + \
                                              [0.0002, 0.0,   0.0] + \
                                                     [0.0002, 0.0] + \
                                                          [0.0005],
        )

    def init_position(self):
        """
        Function for positions that need to be sent at the start of the script
        """
        self.drone.mav.set_gps_global_origin_send(
            target_system=1,
            latitude=43.48,
            longitude=80.53,
            altitude=0
        )
        self.drone.mav.set_home_position_send(
            target_system=1,
            latitude=43.48,
            longitude=80.53,
            altitude=0,
            x=0,
            y=0,
            z=0,
            q = [1,0,0,0],
            approach_x=0,
            approach_y=0,
            approach_z=1,
        )
