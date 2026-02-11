#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="runs/full_flight_$(date +%Y%m%d_%H%M%S)"
BAG_ROOT="$RUN_DIR/bags"
CSV_DIR="$RUN_DIR/csv/drone"
LOG_DIR="$RUN_DIR/logs"

mkdir -p "$BAG_ROOT" "$CSV_DIR" "$LOG_DIR"

# QoS TF
cat > "$RUN_DIR/qos.yaml" <<'YAML'
/tf:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
YAML

export ROS2CLI_DISABLE_DAEMON=1

# sim time mavros
ros2 param set /mavros use_sim_time true || true
ros2 param set /mavros_node use_sim_time true || true

# CSV
ros2 topic echo /mavros/altitude --csv > "$CSV_DIR/altitude.csv" & PID_ALT=$!
ros2 topic echo /mavros/state   --csv > "$CSV_DIR/state.csv"    & PID_STATE=$!
ros2 topic echo /mavros/battery --csv > "$CSV_DIR/battery.csv"  & PID_BATT=$!

PIDS=()
CLEANED=0

MAX_CACHE_SIZE=100000000   # 100MB
MAX_BAG_SIZE=2000000000    # 2GB segments

common_bag_args=(
  --use-sim-time
  --max-cache-size "$MAX_CACHE_SIZE"
  --max-bag-size "$MAX_BAG_SIZE"
  --qos-profile-overrides-path "$RUN_DIR/qos.yaml"
)

start_record() {
  local name="$1"; shift
  local out="$BAG_ROOT/$name"
  local log="$LOG_DIR/${name}.log"

  ros2 bag record -o "$out" \
    "${common_bag_args[@]}" \
    "$@" 2>&1 | tee "$log" &

  PIDS+=("$!")
}


cleanup() {
  # éviter double-exécution (INT + EXIT)
  if [[ "$CLEANED" == "1" ]]; then
    return 0
  fi
  CLEANED=1

  kill $PID_ALT $PID_STATE $PID_BATT 2>/dev/null || true

  if (( ${#PIDS[@]} > 0 )); then
    echo "Stopping recorders..."
    for pid in "${PIDS[@]}"; do
      kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
      wait "$pid" 2>/dev/null || true
    done
  fi
}

trap cleanup INT TERM EXIT

# === Recorders ===

start_record "drone" \
  /iris_depth_camera/rgb_camera/image_raw \
  /iris_depth_camera/rgb_camera/camera_info \
  /iris_depth_camera/depth_camera/points \
  /iris_depth_camera/depth_camera/camera_info \
  /iris_depth_camera/depth_camera/depth/camera_info \
  /mavros/imu/data \
  /mavros/local_position/pose \
  /mavros/altitude \
  /mavros/state \
  /mavros/battery \
  /fmu/out/vehicle_odometry \
  /tf /tf_static /clock

start_record "robot1" \
  /robot1/diff_cont/cmd_vel_unstamped \
  /robot1/diff_cont/odom \
  /robot1/joint_states \
  /robot1/robot_description \
  /scan \
  /camera/image_raw \
  /camera/camera_info \
  /tf /tf_static /clock

start_record "robot2" \
  /robot2/diff_cont/cmd_vel_unstamped \
  /robot2/diff_cont/odom \
  /robot2/joint_states \
  /robot2/robot_description \
  /tf /tf_static /clock

echo "RUN_DIR: $RUN_DIR"
echo "Recording... Press Ctrl+C to stop."
wait
