# Airside SLAM System (Docker)

> [!NOTE]
> Please make sure to create a new branch instead of forking the repo when working on new tasks.

The Docker setup provides:

- ROS 2 Jazzy
- MAVROS
- Cartographer
- sf45b drivers

## Build the image

```sh
./docker/build.sh
```

## Run the container with GUI support

```sh
# Run first terminal
sudo ./docker/run.sh
# or
docker run --rm -it \ 
--privileged \ 
--device /dev/cu.usbmodem38S45_15868 \ 
ros-ws

# Run more terminals
sudo ./docker/attach.sh

# Clear all cache
docker buildx prune --all
```

## Run SLAM commands

```sh
cd /workspace/ros2_ws

rm -rf build install log

colcon build

source install/setup.bash

ros2 launch slam slam_launch.py

# OR

./sf45b

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 laser laser_frame

ros2 run cartographer_ros cartographer_node -configuration_directory /workspace/ros2_ws/src/slam/config -configuration_basename sf45b_2d.lua --ros-args -p provide_odom_frame:=true -p expected_sensor_ids:="[scan]" -r scan:=/scan

ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

rviz2
```

## RUN mirror-pixhawk commands

```sh
cd ros2_ws

rm -rf build install log

# if not on docker (local)
source /opt/ros/jazzy/setup.bash

# should not be needed (this should have already been configured when setting up workspace)
rosdep install -i --from-path src --rosdistro jazzy -y --skip-keys "ament_cmake ament_python"

colcon build
# if multiple packages exist, use:
colcon build --packages-select mirror_pixhawk

source install/local_setup.bash

# node name (currently scan) can be changed in setup.py
ros2 run mirror_pixhawk scan
```

