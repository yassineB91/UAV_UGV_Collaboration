#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/record_comparison_bag.sh [options] [-- <extra ros2 bag args>]

Options:
  -o, --output DIR          Output bag directory.
  -d, --duration SEC        Stop automatically after SEC seconds.
      --message-timeout SEC Wait up to SEC seconds for each required pose topic.
      --with-images         Also record raw camera topics.
      --dry-run             Print the resolved topic list and exit.
      --no-preflight        Skip runtime checks for required pose topics.
  -h, --help                Show this help.

Required live topics:
  /ugv/compare/odom_only_pose
  /ugv/compare/icp_pose
  /ugv/compare/scan_match_pose
  /uav/detection_boxes
  /uav/pose
  /ugv/compare/uav_pose

Examples:
  ./scripts/record_comparison_bag.sh --duration 60
  ./scripts/record_comparison_bag.sh -o /tmp/run01 --message-timeout 20
  ./scripts/record_comparison_bag.sh --duration 90 -- --compression-mode file --compression-format zstd
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"
bag_root="${repo_dir}/bags"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${bag_root}/comparison_${timestamp}"
duration_sec=""
message_timeout_sec=20
with_images=0
dry_run=0
preflight=1
extra_rosbag_args=()

hard_required_topics=(
  "/ugv/compare/odom_only_pose"
  "/ugv/compare/icp_pose"
  "/ugv/compare/scan_match_pose"
  "/uav/detection_boxes"
  "/uav/pose"
  "/ugv/compare/uav_pose"
)

soft_required_topics=()

desired_topics=(
  "/clock"
  "/tf"
  "/tf_static"
  "/map"
  "/uav/obstacle_mask"
  "/uav/detection_boxes"
  "/ugv/scan"
  "/ugv/odom"
  "/ugv/icp/pose"
  "/ugv/scan_match/pose"
  "/uav/pose"
  "/ugv/compare/odom_only_pose"
  "/ugv/compare/odom_only_path"
  "/ugv/compare/icp_pose"
  "/ugv/compare/icp_path"
  "/ugv/compare/scan_match_pose"
  "/ugv/compare/scan_match_path"
  "/ugv/compare/uav_pose"
  "/ugv/compare/uav_path"
)

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found in PATH." >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "timeout command not found in PATH." >&2
  exit 1
fi

has_topic() {
  local wanted="$1"
  local topic
  for topic in "${available_topics[@]}"; do
    if [[ "${topic}" == "${wanted}" ]]; then
      return 0
    fi
  done
  return 1
}

wait_for_topic_message() {
  local topic="$1"
  local wait_sec="$2"

  if timeout "${wait_sec}s" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    return 0
  fi

  echo "Timed out waiting for a message on ${topic} after ${wait_sec}s." >&2
  return 1
}

uav_projection_running_legacy_code() {
  local params
  if ! params="$(ros2 param list /uav_projection_node 2>/dev/null)"; then
    return 1
  fi

  if grep -q '^  uav_pose_projection_mode$' <<<"${params}"; then
    return 1
  fi

  return 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      output_dir="$2"
      shift 2
      ;;
    -d|--duration)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      duration_sec="$2"
      shift 2
      ;;
    --message-timeout)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      message_timeout_sec="$2"
      shift 2
      ;;
    --with-images)
      with_images=1
      shift
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    --no-preflight)
      preflight=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      extra_rosbag_args=("$@")
      break
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

mkdir -p "$(dirname "${output_dir}")"

mapfile -t available_topics < <(ros2 topic list | sort)

record_topics=()
missing_required=()
for topic in "${hard_required_topics[@]}"; do
  if has_topic "${topic}"; then
    record_topics+=("${topic}")
  else
    missing_required+=("${topic}")
  fi
done

for topic in "${soft_required_topics[@]}"; do
  if has_topic "${topic}"; then
    record_topics+=("${topic}")
  else
    echo "Warning: optional comparison topic is not currently available: ${topic}" >&2
  fi
done

if [[ ${#missing_required[@]} -gt 0 ]]; then
  echo "Missing required comparison topics:" >&2
  printf '  %s\n' "${missing_required[@]}" >&2
  exit 1
fi

for topic in "${desired_topics[@]}"; do
  if has_topic "${topic}"; then
    skip=0
    for existing in "${record_topics[@]}"; do
      if [[ "${existing}" == "${topic}" ]]; then
        skip=1
        break
      fi
    done
    if (( ! skip )); then
      record_topics+=("${topic}")
    fi
  fi
done

if (( with_images )); then
  for topic in "/uav/camera/image_raw" "/ugv/camera/image_raw" "/camera/image_raw"; do
    if has_topic "${topic}"; then
      record_topics+=("${topic}")
    fi
  done
fi

topic_manifest="${output_dir}_topics.txt"
{
  echo "# Comparison bag topic manifest"
  echo "# Generated: $(date --iso-8601=seconds)"
  echo "# Output dir: ${output_dir}"
  echo "# Duration seconds: ${duration_sec:-<manual stop>}"
  echo "# Message timeout seconds: ${message_timeout_sec}"
  echo "# Preflight enabled: ${preflight}"
  if [[ ${#extra_rosbag_args[@]} -gt 0 ]]; then
    echo "# Extra ros2 bag args: ${extra_rosbag_args[*]}"
  else
    echo "# Extra ros2 bag args: <none>"
  fi
  echo
  printf '%s\n' "${record_topics[@]}"
} > "${topic_manifest}"

echo "Comparison bag output: ${output_dir}"
echo "Topic manifest: ${topic_manifest}"
echo "Recorded topics:"
printf '  %s\n' "${record_topics[@]}"

if (( dry_run )); then
  exit 0
fi

if (( preflight )); then
  echo "Checking required pose topics..."
  if uav_projection_running_legacy_code; then
    echo "Preflight failed: /uav_projection_node is still running an old build with legacy buffered UAV pose logic." >&2
    echo "Restart /uav_projection_node so /uav/pose uses the restored current-state projection code." >&2
    exit 1
  fi
  for topic in "${hard_required_topics[@]}"; do
    if ! wait_for_topic_message "${topic}" "${message_timeout_sec}"; then
      echo "Preflight failed: ${topic} is present but not publishing usable messages." >&2
      exit 1
    fi
    echo "  OK ${topic}"
  done

  for topic in "${soft_required_topics[@]}"; do
    if ! has_topic "${topic}"; then
      continue
    fi
    if wait_for_topic_message "${topic}" "${message_timeout_sec}"; then
      echo "  OK ${topic}"
    else
      echo "Warning: ${topic} did not publish within ${message_timeout_sec}s; recording will continue." >&2
    fi
  done
fi

record_cmd=(ros2 bag record -o "${output_dir}")
record_cmd+=("${extra_rosbag_args[@]}")
record_cmd+=("${record_topics[@]}")

if [[ -n "${duration_sec}" ]]; then
  echo "Recording for ${duration_sec}s..."
  set +e
  timeout --signal=SIGINT --kill-after=10 "${duration_sec}s" "${record_cmd[@]}"
  status=$?
  set -e
  if [[ ${status} -ne 0 && ${status} -ne 124 && ${status} -ne 130 ]]; then
    exit "${status}"
  fi
  echo "Recording finished."
else
  exec "${record_cmd[@]}"
fi
