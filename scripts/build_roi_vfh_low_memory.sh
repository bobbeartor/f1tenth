#!/usr/bin/env bash
# Run from a terminal with /opt/ros/humble/setup.bash sourced.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$repo_dir"

if ! command -v colcon >/dev/null 2>&1; then
  echo 'colcon not found. Source /opt/ros/humble/setup.bash first.' >&2
  exit 1
fi

# colcon limits package concurrency; Make/CMake limits compiler concurrency.
# Both are needed: sequential packages alone can still run many cc1plus jobs.
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS='-j1'

exec colcon build \
  --executor sequential \
  --packages-up-to roi_vfh_test \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
