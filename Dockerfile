FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ==========================================
# VERSION CONFIGURATION (pin)
# ==========================================
ARG PX4_VERSION=v1.15.2
ARG PX4_MSGS_BRANCH=release/1.15
ARG PX4_ROS_COM_BRANCH=release/1.15
ARG XRCE_AGENT_VERSION=v2.4.3

# Pin symforce (PX4 build dependency: symforce.symbolic)
ARG SYMFORCE_VERSION=0.10.1

# ==========================================
# APT MIRRORS (HTTPS)
# ==========================================
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list

# ==========================================
# BASE DEPENDENCIES
# ==========================================
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    wget curl gnupg2 lsb-release \
    git cmake build-essential ninja-build \
    python3-pip python3-dev python3-jinja2 python3-numpy python3-yaml \
    python3-empy python3-setuptools python3-toml \
    python3-pygments python3-kconfiglib python3-packaging \
    python3-pexpect python3-future python3-serial python3-lxml \
    protobuf-compiler libprotobuf-dev libeigen3-dev libopencv-dev \
    libasio-dev libtinyxml2-dev libssl-dev && \
    rm -rf /var/lib/apt/lists/*

#==============================================

# Empêche Ultralytics d'auto-installer / upgrader des deps (lap, numpy, etc.)
ENV ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS=1




# ==========================================
# ==========================================
# SYMFORCE (needed by PX4 build: symforce.symbolic) - pinned
# ==========================================
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --ignore-installed "symforce==${SYMFORCE_VERSION}"



# ==========================================
# REMOVE GZ (Ignition/Garden/Fortress/Harmonic) IF PRESENT
# (avoid conflicts with Gazebo Classic)
# ==========================================
RUN set -eux; \
    apt-get update && \
    apt-get remove -y \
        gz-garden \
        gz-fortress \
        gz-harmonic \
        gz-tools2 \
        libgz-* \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-image \
        ros-humble-ros-gz-interfaces \
        || true && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ==========================================
# GAZEBO CLASSIC 11 (gzserver/gzclient)
# ==========================================
RUN set -eux; \
    apt-get update \
    apt-get install -y gazebo libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# ==========================================
# ROS 2 PACKAGES (GAZEBO CLASSIC) ✅ PERMANENT
# Includes what you installed manually:
# - ros-humble-gazebo-ros-pkgs
# - ros-humble-gazebo-ros2-control (brings libgazebo_ros2_control.so)
# ==========================================
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    \
    # Gazebo ROS (Classic)
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    \
    # ros2_control + gazebo_ros2_control plugin
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-navigation2 \
    \
    ros-humble-robot-state-publisher \
    ros-humble-image-transport-plugins \
    ros-humble-rqt-image-view \
    ros-humble-rviz2 \
    ros-humble-twist-mux \
    ros-humble-tf2-tools \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs-py \
    ros-humble-vision-msgs && \
    rm -rf /var/lib/apt/lists/*

# ==========================================
# ✅ MAVROS (ROS2) + GeographicLib datasets (permanent)
# This avoids doing it manually in the container.
# ==========================================
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    geographiclib-tools && \
    rm -rf /var/lib/apt/lists/*

# MAVROS needs GeographicLib datasets (EGM96 etc.)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh || true

# ==========================================
# GSTREAMER (PX4 cameras)
# ==========================================
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav && \
    rm -rf /var/lib/apt/lists/*

# ==========================================
# MICRO-XRCE-DDS-AGENT (SUPERBUILD=ON, v2.x)
# ==========================================
WORKDIR /root
RUN git clone -b ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake -DUAGENT_SUPERBUILD=ON \
          -DUAGENT_BUILD_EXECUTABLE=ON \
          -DCMAKE_INSTALL_PREFIX=/opt/microxrce \
          -DUAGENT_P2P_PROFILE=OFF \
          .. && \
    cmake --build . -j$(nproc) && \
    cmake --install . && \
    echo "/opt/microxrce/lib" > /etc/ld.so.conf.d/microxrce.conf && \
    ldconfig && \
    cd /root && rm -rf Micro-XRCE-DDS-Agent

# ==========================================
# PX4 AUTOPILOT (PINNED)
# ==========================================
WORKDIR /root
RUN git clone --branch ${PX4_VERSION} --recursive \
    https://github.com/PX4/PX4-Autopilot.git

WORKDIR /root/PX4-Autopilot

# Fix invalid pip requirement syntax for newer pip versions
RUN sed -i 's/matplotlib>=3.0.*/matplotlib>=3.0/' ./Tools/setup/requirements.txt

RUN bash ./Tools/setup/ubuntu.sh --no-nuttx

# ==========================================
# BUILD PX4 SITL (to get Gazebo Classic plugins)
# ==========================================
WORKDIR /root/PX4-Autopilot
RUN DONT_RUN=1 make px4_sitl gazebo-classic

# ==========================================
# GUI SUPPORT FOR WSL2
# ==========================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# ==========================================
# ROS 2 WORKSPACE (px4_msgs + px4_ros_com aligned)
# ==========================================
WORKDIR /root/dev_ws
RUN mkdir -p src

WORKDIR /root/dev_ws/src
RUN git clone --branch ${PX4_MSGS_BRANCH} https://github.com/PX4/px4_msgs.git && \
    git clone --branch ${PX4_ROS_COM_BRANCH} https://github.com/PX4/px4_ros_com.git

RUN source /opt/ros/humble/setup.bash && \
    ros2 pkg create my_package --build-type ament_cmake

RUN mkdir -p /root/dev_ws/src/my_package/{urdf,launch,worlds,config,models}

WORKDIR /root/dev_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# ==========================================
# ENVIRONMENT VARIABLES
# ==========================================
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0

ENV PATH="/opt/microxrce/bin:${PATH}"
ENV LD_LIBRARY_PATH="/opt/microxrce/lib:${LD_LIBRARY_PATH}"

# Gazebo Classic paths (include PX4 compiled plugins)
ENV GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/root/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic"
ENV GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/models:/root/dev_ws/src/my_package/models:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
ENV GAZEBO_RESOURCE_PATH="/root/PX4-Autopilot/Tools/simulation/gazebo-classic:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/worlds:/root/dev_ws/src/my_package/worlds"

ENV GAZEBO_MASTER_URI=http://127.0.0.1:11345
ENV GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org

# ==========================================
# BASHRC CONFIGURATION
# ==========================================
RUN echo "" >> ~/.bashrc && \
    echo "# ROS 2 Humble" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /root/dev_ws/install/setup.bash ]; then source /root/dev_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Gazebo Classic 11" >> ~/.bashrc && \
    echo "if [ -f /usr/share/gazebo/setup.sh ]; then source /usr/share/gazebo/setup.sh; fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Micro XRCE Agent libs" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/microxrce/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "# Gazebo plugin path (ROS + system + PX4)" >> ~/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc

# ==========================================
# Ultralytics YOLO (and deps)
# ==========================================
RUN python3 -m pip install --no-cache-dir --break-system-packages ultralytics


#============================================
# Fix YOLO deps: garder numpy 1.x (cv_bridge) + scipy/lap + sympy compatible torch
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    numpy==1.26.4 \
    scipy==1.11.4 \
    lap==0.5.12 \
    "sympy>=1.12,<1.14"

# ==========================================
# ENTRYPOINT
# ==========================================
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /root/dev_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
