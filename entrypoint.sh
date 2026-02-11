#!/bin/bash
set -eo pipefail

echo "=============================="
echo " PX4 + Gazebo Classic + ROS2 "
echo "=============================="

# -----------------------------
# Defaults (safe)
# -----------------------------
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

AUTOSTART_PX4="${AUTOSTART_PX4:-0}"
AUTOSTART_ROS2="${AUTOSTART_ROS2:-0}"
AUTOSTART_MAVROS="${AUTOSTART_MAVROS:-0}"

# Headless by default (WSLg gzclient often crashes)
HEADLESS="${HEADLESS:-1}"

XRCE_TRANSPORT="${XRCE_TRANSPORT:-udp}"
XRCE_UDP_PORT="${XRCE_UDP_PORT:-8888}"
XRCE_TCP_PORT="${XRCE_TCP_PORT:-8888}"

# MAVROS defaults
MAVROS_FCU_URL="${MAVROS_FCU_URL:-udp://:14540@127.0.0.1:14557}"
MAVROS_SYS_ID="${MAVROS_SYS_ID:-1}"
MAVROS_COMP_ID="${MAVROS_COMP_ID:-1}"
MAVROS_PROTOCOL="${MAVROS_PROTOCOL:-v2.0}"

# -----------------------------
# 1) Source Gazebo Classic
# -----------------------------
# NOTE: On your system /usr/share/gazebo/setup.sh exists, /usr/share/gazebo-11/setup.sh often doesn't.
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
elif [ -f /usr/share/gazebo-11/setup.sh ]; then
  source /usr/share/gazebo-11/setup.sh
fi

# -----------------------------
# 2) Source ROS2 + workspace
# -----------------------------
source /opt/ros/humble/setup.bash
if [ -f /root/dev_ws/install/setup.bash ]; then
  source /root/dev_ws/install/setup.bash
fi

# Add sourcing to bashrc for interactive shells
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc 2>/dev/null; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "[INFO] Added ROS2 Humble sourcing to ~/.bashrc"
fi

if ! grep -q "source ~/dev_ws/install/setup.bash" ~/.bashrc 2>/dev/null; then
  echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
  echo "[INFO] Added dev_ws sourcing to ~/.bashrc"
fi

# -----------------------------
# 3) Ensure Gazebo paths are sane (avoid duplicates)
# -----------------------------
PX4_GZ_BUILD="/root/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic"
PX4_GZ_MODELS="/root/PX4-Autopilot/Tools/simulation/gazebo-classic/models"
PX4_GZ_SITL_MODELS="/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
MY_MODELS="/root/dev_ws/src/my_package/models"
MY_WORLDS="/root/dev_ws/src/my_package/worlds"
GZ_SYS_PLUGINS="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
GZ_SYS_MODELS="/usr/share/gazebo-11/models"

# Include /opt/ros/humble/lib to allow Gazebo to find libgazebo_ros2_control.so
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GZ_SYS_PLUGINS}:${PX4_GZ_BUILD}"
export GAZEBO_MODEL_PATH="${MY_MODELS}:${PX4_GZ_SITL_MODELS}:${PX4_GZ_MODELS}:${GZ_SYS_MODELS}"


# Resource path: include PX4 gazebo-classic base + worlds + your worlds
export GAZEBO_RESOURCE_PATH="/root/PX4-Autopilot/Tools/simulation/gazebo-classic:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/worlds:${MY_WORLDS}"

# Master URI default
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

# -----------------------------
# 4) Print useful config
# -----------------------------
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
echo "GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}"
echo "HEADLESS=${HEADLESS}"
echo "XRCE_TRANSPORT=${XRCE_TRANSPORT}"
echo "AUTOSTART_MAVROS=${AUTOSTART_MAVROS}"
echo "MAVROS_FCU_URL=${MAVROS_FCU_URL}"

# -----------------------------
# 5) Optional: rebuild workspace (dev mode)
# -----------------------------
if [ -d /root/dev_ws/src/my_package ]; then
  echo "[INFO] colcon build (dev mode)..."
  cd /root/dev_ws
  colcon build --symlink-install || true
  source /root/dev_ws/install/setup.bash || true
fi

# -----------------------------
# 6) Start Micro XRCE Agent automatically (idempotent)
# -----------------------------
if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  if [ "$XRCE_TRANSPORT" = "udp" ]; then
    echo "[INFO] Starting MicroXRCEAgent UDP on port ${XRCE_UDP_PORT}, domain ${ROS_DOMAIN_ID}"
    MicroXRCEAgent udp4 -p "${XRCE_UDP_PORT}" -d "${ROS_DOMAIN_ID}" &
  elif [ "$XRCE_TRANSPORT" = "tcp" ]; then
    echo "[INFO] Starting MicroXRCEAgent TCP on port ${XRCE_TCP_PORT}, domain ${ROS_DOMAIN_ID}"
    MicroXRCEAgent tcp4 -p "${XRCE_TCP_PORT}" -d "${ROS_DOMAIN_ID}" &
  else
    echo "[WARN] XRCE_TRANSPORT=${XRCE_TRANSPORT} not recognized. Skipping agent start."
  fi
else
  echo "[INFO] MicroXRCEAgent already running."
fi

# -----------------------------
# 7) Optional: start PX4 + Gazebo Classic automatically
# -----------------------------
if [ "${AUTOSTART_PX4}" = "1" ]; then
  echo "[INFO] AUTOSTART_PX4=1 -> starting PX4 SITL + Gazebo Classic (HEADLESS=${HEADLESS})"

  # Best-effort cleanup (avoid "already running" / stale pids)
  pkill -f "px4.*px4_sitl_default" >/dev/null 2>&1 || true
  pkill -f gzserver >/dev/null 2>&1 || true
  pkill -f gazebo >/dev/null 2>&1 || true
  sleep 1

  cd /root/PX4-Autopilot

  if [ "${HEADLESS}" = "1" ]; then
    HEADLESS=1 make px4_sitl gazebo-classic &
  else
    make px4_sitl gazebo-classic &
  fi

  # Give PX4 time to open MAVLink ports
  sleep 3
fi

# -----------------------------
# 8) Optional: start ROS2 launch automatically
# -----------------------------
if [ "${AUTOSTART_ROS2}" = "1" ]; then
  PKG="${ROS2_LAUNCH_PKG:-my_package}"
  FILE="${ROS2_LAUNCH_FILE:-bringup.launch.py}"
  echo "[INFO] AUTOSTART_ROS2=1 -> ros2 launch ${PKG} ${FILE}"
  ros2 launch "${PKG}" "${FILE}" &
fi

# -----------------------------
# 9) Optional: start MAVROS automatically (idempotent)
# -----------------------------
if [ "${AUTOSTART_MAVROS}" = "1" ]; then
  if pgrep -f "mavros_node" >/dev/null 2>&1; then
    echo "[INFO] MAVROS already running."
  else
    echo "[INFO] AUTOSTART_MAVROS=1 -> starting mavros_node"
    echo "[INFO] fcu_url=${MAVROS_FCU_URL}"
    ros2 run mavros mavros_node --ros-args \
      -p fcu_url:="${MAVROS_FCU_URL}" \
      -p target_system_id:="${MAVROS_SYS_ID}" \
      -p target_component_id:="${MAVROS_COMP_ID}" \
      -p fcu_protocol:="${MAVROS_PROTOCOL}" &
  fi
fi

# -----------------------------
# 10) Final command
# -----------------------------
echo "[INFO] Container ready. Running: $*"
exec "$@"
