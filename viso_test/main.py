import time
import depthai as dai
import numpy as np
import math

USE_RERUN = False
USE_MAVLINK = True
USE_VISUAL_POSITION = False

if USE_MAVLINK:
    from drone_connection import DroneConnection
    success, drone_connection = DroneConnection.create()
    if not success:
        print("Failed to connect to drone")
        exit(1)

if USE_RERUN:
    from rerun_node import RerunNode

# functions
def quat_to_euler(qw, qx, qy, qz):
    roll = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
    pitch = math.asin(2*(qw*qy - qz*qx))
    yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
    return -roll, pitch, -yaw

# Create pipeline

try:
    with dai.Pipeline() as p:
        fps = 60
        width = 640
        height = 400
        # Define sources and outputs
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        imu = p.create(dai.node.IMU)
        odom = p.create(dai.node.BasaltVIO)
        slam = p.create(dai.node.RTABMapSLAM)
        stereo = p.create(dai.node.StereoDepth)
        params = {"RGBD/CreateOccupancyGrid": "true",
                "Grid/3D": "true",
                "Rtabmap/SaveWMState": "true"}
        slam.setParams(params)

        if USE_RERUN:
            rerunViewer = RerunNode()
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER, dai.IMUSensor.GYROSCOPE_CALIBRATED], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        stereo.setExtendedDisparity(False)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)
        stereo.initialConfig.setLeftRightCheckThreshold(10)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)


        left.requestOutput((width, height)).link(stereo.left)
        right.requestOutput((width, height)).link(stereo.right)
        stereo.syncedLeft.link(odom.left)
        stereo.syncedRight.link(odom.right)
        stereo.depth.link(slam.depth)
        stereo.rectifiedLeft.link(slam.rect)
        imu.out.link(odom.imu)

        odom.transform.link(slam.odom)

        if USE_RERUN:
            slam.transform.link(rerunViewer.inputTrans)
            slam.passthroughRect.link(rerunViewer.inputImg)
            slam.occupancyGridMap.link(rerunViewer.inputGrid)
            slam.obstaclePCL.link(rerunViewer.inputObstaclePCL)
            slam.groundPCL.link(rerunViewer.inputGroundPCL)

        slamQueue = slam.transform.createOutputQueue(maxSize=4, blocking=False)
        odomQueue = odom.transform.createOutputQueue(maxSize=4, blocking=False)
        imuQueue = imu.out.createOutputQueue(maxSize=4, blocking=False)

        p.start()
        if USE_MAVLINK and drone_connection is not None:
            drone_connection.init_position()


        while p.isRunning():
            odomData = odomQueue.tryGet()
            if odomData is not None:
                print("Odom:")
                #print(slamData)
                print("-----")
                print(f"X -> Forward: {-odomData.getTranslation().x}")
                print(f"Y -> Right: {odomData.getTranslation().y}")
                print(f"Z -> Down: {-odomData.getTranslation().z}")

                if USE_MAVLINK:
                    x = -odomData.getTranslation().x
                    y = odomData.getTranslation().y
                    z = -odomData.getTranslation().z
                    qw = odomData.getQuaternion().qw
                    qx = odomData.getQuaternion().qx
                    qy = odomData.getQuaternion().qy
                    qz = odomData.getQuaternion().qz
                    
                    roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
                    print("-----")
                    print(f"Roll -> Right: {roll}")
                    print(f"Pitch -> Up: {pitch}")
                    print(f"Yaw -> Right: {yaw}")
                    
                    if USE_VISUAL_POSITION:
                        roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
                        drone_connection.send_vision_position_estimate(x, y, z, roll, pitch, yaw)
                    else:
                        # TODO: Do these please
                        vx = vy = vz = 0.0

                        imuData = imuQueue.tryGet()
                        if imuData is not None:
                            gyro = imuData.packets[-1].gyroscope
                            angular_vx = gyro.z
                            angular_vy = gyro.x
                            angular_vz = gyro.y
                        else:
                            angular_vx = angular_vy = angular_vz = 0.0

                        drone_connection.send_odometry(
                            x, y, z,
                            [qw, qx, qy, qz],
                            vx, vy, vz,
                            angular_vx, angular_vy, angular_vz)
                time.sleep(0.05)
except Exception as e:
    print(f"Error: {e}")
    exit(1)
