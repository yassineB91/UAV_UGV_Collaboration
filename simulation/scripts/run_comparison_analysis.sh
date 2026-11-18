#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_comparison_analysis.sh BAG_DIR [OUTPUT_DIR] [-- <extra analyze args>]

Examples:
  ./scripts/run_comparison_analysis.sh /root/dev_ws/src/my_package/bags/comparison_20260623_110312
  ./scripts/run_comparison_analysis.sh /root/dev_ws/src/my_package/bags/comparison_20260623_110312 /root/dev_ws/src/my_package/analysis/comparison_20260623_110312
  ./scripts/run_comparison_analysis.sh /root/dev_ws/src/my_package/bags/comparison_20260623_110312 -- --sync-tolerance-ms 300
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 1
fi

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

bag_dir="$1"
shift

output_dir=""
extra_args=()

if [[ $# -gt 0 && "$1" != "--" ]]; then
  output_dir="$1"
  shift
fi

if [[ $# -gt 0 ]]; then
  if [[ "$1" != "--" ]]; then
    echo "Unexpected argument: $1" >&2
    usage >&2
    exit 1
  fi
  shift
  extra_args=("$@")
fi

if [[ ! -d "${bag_dir}" ]]; then
  echo "Bag directory not found: ${bag_dir}" >&2
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cmd=(python3 "${script_dir}/analyze_comparison_bag.py" "${bag_dir}")
if [[ -n "${output_dir}" ]]; then
  cmd+=(-o "${output_dir}")
fi
cmd+=("${extra_args[@]}")

echo "Running analysis command:"
printf '  %q' "${cmd[@]}"
printf '\n'

exec "${cmd[@]}"
