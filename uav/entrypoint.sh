#!/bin/bash
set -eo pipefail

workspace=/root/dev_ws
package_dir="$workspace/src/uav"
workspace_setup="$workspace/install/setup.bash"

ros_setup=
for setup in /opt/ros/*/setup.bash; do
  if [ -f "$setup" ]; then
    ros_setup="$setup"
    break
  fi
done

if [ -z "$ros_setup" ]; then
  echo "Installation ROS introuvable" >&2
  exit 1
fi

source "$ros_setup"

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  apt-get update
  apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
fi

mkdir -p "$package_dir"/{urdf,launch,worlds,config,models,scripts}

if [ ! -f "$package_dir/package.xml" ]; then
  temp_dir="$(mktemp -d)"
  trap 'rm -rf -- "$temp_dir"' EXIT
  (
    cd "$temp_dir"
    ros2 pkg create --build-type ament_cmake uav
  )
  cp "$temp_dir/uav/package.xml" "$package_dir/package.xml"
  rm -rf -- "$temp_dir"
  trap - EXIT
fi

if [ -f "$workspace_setup" ]; then
  source "$workspace_setup"
fi

ros_init="source $ros_setup"
workspace_init="[ ! -f $workspace_setup ] || source $workspace_setup"

if ! grep -Fxq "$ros_init" "$HOME/.bashrc" 2>/dev/null; then
  printf '\n%s\n' "$ros_init" >> "$HOME/.bashrc"
fi

if ! grep -Fxq "$workspace_init" "$HOME/.bashrc" 2>/dev/null; then
  printf '%s\n' "$workspace_init" >> "$HOME/.bashrc"
fi

python3 -m pip install smbus2 'eclipse-zenoh==1.5.1'

exec python3 -u /app/subscriber.py