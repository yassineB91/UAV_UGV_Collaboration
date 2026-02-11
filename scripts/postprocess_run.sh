#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <RUN_DIR> [--move-csv-from <CSV_SOURCE_DIR>]"
  echo "Example: $0 runs/full_flight_20260107_141500"
  exit 1
fi

RUN_DIR="$1"
BAG_ROOT="$RUN_DIR/bags"

if [[ ! -d "$RUN_DIR" ]]; then
  echo "ERROR: RUN_DIR not found: $RUN_DIR"
  exit 1
fi

if ! command -v zstd >/dev/null 2>&1; then
  echo "ERROR: zstd not installed. Install it with:"
  echo "  sudo apt-get update && sudo apt-get install -y zstd"
  exit 1
fi

decompress_one_dir() {
  local d="$1"
  [[ -d "$d" ]] || return 0

  shopt -s nullglob
  local files=("$d"/*.db3.zstd)
  shopt -u nullglob

  if (( ${#files[@]} == 0 )); then
    echo "No .db3.zstd in: $d"
    return 0
  fi

  echo "Decompressing in: $d"
  ( cd "$d" && zstd -d --keep *.db3.zstd )
}

echo "RUN_DIR: $RUN_DIR"
echo "BAG_ROOT: $BAG_ROOT"

decompress_one_dir "$BAG_ROOT/drone"
decompress_one_dir "$BAG_ROOT/robot1"
decompress_one_dir "$BAG_ROOT/robot2"

# Optionnel: déplacer des CSV depuis un autre dossier vers RUN_DIR/csv/drone
if [[ $# -ge 3 && "$2" == "--move-csv-from" ]]; then
  CSV_SRC="$3"
  mkdir -p "$RUN_DIR/csv/drone"
  shopt -s nullglob
  csvs=("$CSV_SRC"/*.csv)
  shopt -u nullglob
  if (( ${#csvs[@]} > 0 )); then
    echo "Copying CSV from $CSV_SRC -> $RUN_DIR/csv/drone/"
    cp -a "$CSV_SRC"/*.csv "$RUN_DIR/csv/drone/" || true
  else
    echo "No CSV found in: $CSV_SRC"
  fi
fi

echo "Done."
echo "You can now run:"
echo "  ros2 bag info  $BAG_ROOT/drone"
echo "  ros2 bag play  $BAG_ROOT/drone --clock"
