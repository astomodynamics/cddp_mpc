# Multi-stage Dockerfile for cddp_mpc with ROS 2 Humble + PX4 SITL tooling.

ARG CUDA_VERSION=12.3.2
ARG UBUNTU_VERSION=22.04
ARG ROS_DISTRO=humble
ARG MICRO_XRCE_AGENT_TAG=v2.4.3

FROM nvidia/cuda:${CUDA_VERSION}-cudnn9-devel-ubuntu${UBUNTU_VERSION} AS base

ARG ROS_DISTRO
ARG MICRO_XRCE_AGENT_TAG
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    git \
    build-essential \
    cmake \
    ninja-build \
    wget \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-launch-ros \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    gz-garden \
    ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    empy \
    pyros-genmsg \
    setuptools \
    kconfiglib \
    jsonschema \
    jinja2

RUN git clone --depth 1 --branch ${MICRO_XRCE_AGENT_TAG} \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent && \
    cmake -S /tmp/Micro-XRCE-DDS-Agent -B /tmp/Micro-XRCE-DDS-Agent/build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/Micro-XRCE-DDS-Agent/build --parallel "$(nproc)" && \
    install -Dm0755 /tmp/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent /usr/local/bin/MicroXRCEAgent && \
    find /tmp/Micro-XRCE-DDS-Agent/build -type f \
      \( -name 'libmicroxrcedds_agent.so*' -o -name 'libmicroxrcedds_client.so*' -o -name 'libmicrocdr.so*' -o -name 'libfastrtps.so*' -o -name 'libfastcdr.so*' \) \
      -exec install -Dm0755 {} /usr/local/lib/ \; && \
    ldconfig /usr/local/lib/ && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15 /opt/PX4-Autopilot && \
    cd /opt/PX4-Autopilot && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools && \
    DONT_RUN=1 make px4_sitl_default && \
    chmod -R a+rw /opt/PX4-Autopilot

ENV PX4_HOME=/opt/PX4-Autopilot
ENV PATH="${PX4_HOME}/Tools:${PATH}"
ENV GZ_SIM_RESOURCE_PATH="${PX4_HOME}/Tools/simulation/gz/models"
ENV GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_HOME}/build/px4_sitl_default/build_gz"

RUN rosdep init || true && rosdep update --rosdistro ${ROS_DISTRO}

FROM base AS dev

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    apt-get update && apt-get install -y sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USERNAME}/ws
RUN mkdir -p src && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}/ws/src/cddp_mpc

WORKDIR /home/${USERNAME}/ws/src
RUN git clone https://github.com/PX4/px4_msgs.git -b release/1.15

WORKDIR /home/${USERNAME}/ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select px4_msgs cddp_mpc \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /home/${USERNAME}/ws/install/setup.bash ]; then source /home/${USERNAME}/ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    echo "export PX4_HOME=/opt/PX4-Autopilot" >> /home/${USERNAME}/.bashrc && \
    echo "export PATH=\"\${PX4_HOME}/Tools:\${PATH}\"" >> /home/${USERNAME}/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=\"\${PX4_HOME}/Tools/simulation/gz/models:\${GZ_SIM_RESOURCE_PATH}\"" >> /home/${USERNAME}/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\"\${PX4_HOME}/build/px4_sitl_default/build_gz:\${GZ_SIM_SYSTEM_PLUGIN_PATH}\"" >> /home/${USERNAME}/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown ${USERNAME}:${USERNAME} /entrypoint.sh

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws/src/cddp_mpc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM base AS prod

ARG USERNAME=appuser
ARG USER_UID=1001
ARG USER_GID=1001

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

WORKDIR /app/ros_ws/src
COPY --chown=${USERNAME}:${USERNAME} . /app/ros_ws/src/cddp_mpc

RUN git clone https://github.com/PX4/px4_msgs.git -b release/1.15 /app/ros_ws/src/px4_msgs && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /app/ros_ws && \
    colcon build --packages-select px4_msgs cddp_mpc \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    chown -R ${USERNAME}:${USERNAME} /app

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown ${USERNAME}:${USERNAME} /entrypoint.sh

USER ${USERNAME}
WORKDIR /app/ros_ws/src/cddp_mpc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
