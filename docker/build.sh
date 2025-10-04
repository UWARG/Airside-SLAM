#!/usr/bin/env bash
set -e

IMAGE_NAME="ros-ws"
LIDAR_PORT="${1:-/dev/ttyAMA0}"
LIDAR_RANGE="${2:-320}"

docker build -t $IMAGE_NAME \
    --build-arg LIDAR_PORT="$LIDAR_PORT" \
    --build-arg LIDAR_RANGE="$LIDAR_RANGE" \
    -f docker/Dockerfile .
