#!/usr/bin/env bash
set -e

IMAGE_NAME="ros-ws"
CONTAINER_NAME="ros-container"

if [ -z "${DISPLAY:-}" ]; then
    if getent hosts host.docker.internal >/dev/null 2>&1; then
        export DISPLAY=host.docker.internal:0
    else
        export DISPLAY=:0
    fi
fi

# Base Docker arguments
DOCKER_ARGS=(
    -it
    --name "$CONTAINER_NAME"
    --rm
    --ipc host
    --pid host
    --env QT_X11_NO_MITSHM=1
    --env DISPLAY
    --network host
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --volume "$(pwd):/workspace"
)

SERIAL_GLOBS=(/dev/ttyAMA0 /dev/serial0 /dev/ttyS0 /dev/ttyUSB* /dev/ttyACM*)
for glob in "${SERIAL_GLOBS[@]}"; do
    for dev in $glob; do
        if [ -e "$dev" ]; then
            DOCKER_ARGS+=(--device "$dev:$dev")
        fi
    done
done

if [ -n "${SERIAL_DEVICES:-}" ]; then
    for dev in ${SERIAL_DEVICES}; do
        if [ -e "$dev" ]; then
            DOCKER_ARGS+=(--device "$dev:$dev")
        else
            echo "[warn] Requested serial device $dev not found on host" >&2
        fi
    done
fi

echo "Launching container with serial devices passed through:"
printf '  %s\n' "${DOCKER_ARGS[@]}" | grep -- '--device' || echo "  (none detected)"

xhost +local:docker > /dev/null 2>&1 || true

if docker ps -aq -f name=^${CONTAINER_NAME}$ > /dev/null 2>&1; then
    if [[ -n "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]]; then
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
    fi
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -lc "\
    cp /opt/sf45-driver/build/sf45b /workspace/sf45b 2>/dev/null || true; \
    chmod +x /workspace/sf45b 2>/dev/null || true; \
    exec bash"
