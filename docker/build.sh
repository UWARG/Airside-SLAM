#!/usr/bin/env bash
set -e

IMAGE_NAME="ros-ws"

docker build -t $IMAGE_NAME -f docker/Dockerfile .
