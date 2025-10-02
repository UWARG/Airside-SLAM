# The Adityya Special - ROS 2 with MAVROS

The Docker setup provides:

- ROS 2 Jazzy (desktop)

## Build the image

```sh
./docker/build.sh
```

## Run the container with GUI support

```sh
# Run first terminal
sudo ./docker/run.sh


# Run more terminals
sudo ./docker/attach.sh
```

## Run SLAM commands

```sh
./sf45b

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 laser laser_frame

ros2 run cartographer_ros cartographer_node -configuration_directory /workspace/ros2_ws/src/slam/config -configuration_basename sf45b_2d.lua --ros-args -p provide_odom_frame:=true -p expected_sensor_ids:="[scan]" -r scan:=/scan

ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

rviz2
```
