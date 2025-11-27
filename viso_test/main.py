import time
import depthai as dai
import numpy as np

USE_RERUN = False

if USE_RERUN:
    from rerun_node import RerunNode

# Create pipeline

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
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
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
    p.start()
    while p.isRunning():
        slamData = slamQueue.tryGet()
        if slamData is not None:
            print("Odom:")
            print(slamData)
            print("-----")
            print(slamData.getTranslation().x)
            print(slamData.getTranslation().y)
            print(slamData.getTranslation().z)
            print(slamData.getQuaternion().qx)
        time.sleep(0.1)
