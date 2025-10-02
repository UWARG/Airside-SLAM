FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-cartographer-rviz \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    sudo \
    build-essential \
    curl \
    wget \
    gnupg \
    lsb-release \
    vim \
    tmux \
    less \
    net-tools \
    iputils-ping \
    xauth \
    x11-apps \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    libglu1-mesa \
    libxkbcommon-x11-0 \
    python3-tk \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

RUN geographiclib-get-geoids best && geographiclib-get-gravity egm96 && geographiclib-get-magnetic emm2015

RUN rosdep init || true && rosdep update

ARG USERNAME=root
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
        EXISTING_GROUP_NAME="$(getent group ${USER_GID} | cut -d: -f1)"; \
        if [ "${EXISTING_GROUP_NAME}" != "${USERNAME}" ]; then \
            echo "GID ${USER_GID} taken by ${EXISTING_GROUP_NAME}; creating group ${USERNAME} with arbitrary GID"; \
            groupadd "${USERNAME}" || true; \
        fi; \
    else \
        groupadd -g "${USER_GID}" "${USERNAME}"; \
    fi; \
    if id -u "${USER_UID}" >/dev/null 2>&1; then \
        echo "UID ${USER_UID} in use; creating ${USERNAME} with next available UID"; \
        useradd -m -g "${USERNAME}" "${USERNAME}" || true; \
    else \
        useradd -m -u "${USER_UID}" -g "${USERNAME}" "${USERNAME}"; \
    fi; \
    for grp in dialout plugdev tty video; do \
        if getent group "$grp" > /dev/null 2>&1; then \
            usermod -a -G "$grp" "${USERNAME}" || true; \
        fi; \
    done; \
    mkdir -p /workspace/src; \
    chown -R "${USERNAME}:${USERNAME}" /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY --chown=${USERNAME}:${USERNAME} . /workspace

RUN bash -lc 'if [ -d /workspace/ros2_ws/src ]; then cd /workspace/ros2_ws; colcon build || true; fi'

RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone https://github.com/LightWare-Optoelectronics/lightwarelidar2 /opt/sf45-driver && \
    cd /opt/sf45-driver && mkdir -p build && cd build && \
    cmake .. && cmake --build . --parallel $(nproc)"

RUN ln -s /workspace/docker/run_slam.sh /usr/local/bin/run_slam || true

ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc \
 && chown ${USERNAME}:${USERNAME} /root/.bashrc

USER ${USERNAME}

CMD ["bash"]
