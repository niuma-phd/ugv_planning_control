#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
用法：
  scripts/build_gps_subject2.sh [--clean]

说明：
  在独立工作区构建 GPS 单点定位顺序压点链；不会构建或启动 LIO。
EOF
}

CLEAN=false
while (($# > 0)); do
  case "$1" in
    --clean)
      CLEAN=true
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "未找到 ROS 2 环境：${ROS_SETUP}" >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
set -u

if ! command -v colcon >/dev/null 2>&1; then
  echo "未找到 colcon。" >&2
  exit 2
fi

PACKAGES=(
  ugv_localization_mvp
  ugv_subject2_mvp
  ugv_gps_waypoint_control
)
BUILD_BASE="${REPOSITORY_ROOT}/build_gps"
INSTALL_BASE="${REPOSITORY_ROOT}/install_gps"
LOG_BASE="${REPOSITORY_ROOT}/log_gps"

if [[ "${CLEAN}" == true ]]; then
  rm -rf -- "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
fi

cd "${REPOSITORY_ROOT}"
colcon \
  --log-base "${LOG_BASE}" \
  build \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --executor sequential \
  --packages-select "${PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "GPS 顺序压点构建完成：source ${INSTALL_BASE}/setup.bash"
