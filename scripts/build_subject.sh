#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
用法：
  scripts/build_subject1.sh [--clean]
  scripts/build_subject2.sh [--clean]

选项：
  --clean    构建前删除本科目的 build/install/log 目录
  -h, --help 显示帮助

说明：
  默认使用 ROS 2 Humble 和 Release 构建类型。
  两个科目使用相互独立的构建、安装和日志目录。
EOF
}

SUBJECT="${1:-}"
if [[ "${SUBJECT}" != "subject1" && "${SUBJECT}" != "subject2" ]]; then
  usage >&2
  exit 2
fi
shift

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

# ROS 的 setup.bash 不能在 nounset 开启时可靠加载。
set +u
source "${ROS_SETUP}"
set -u

if ! command -v colcon >/dev/null 2>&1; then
  echo "未找到 colcon，请先安装 python3-colcon-common-extensions。" >&2
  exit 2
fi

case "${SUBJECT}" in
  subject1)
    LABEL="科目一"
    PACKAGES=(
      ugv_localization_mvp
      ugv_subject1_perception_mvp
      ugv_subject1_avoidance_mvp
      ugv_subject1_bringup
    )
    ;;
  subject2)
    LABEL="科目二"
    PACKAGES=(
      ugv_localization_mvp
      ugv_subject2_mvp
      ugv_subject2_bringup
    )
    ;;
esac

BUILD_BASE="${REPOSITORY_ROOT}/build_${SUBJECT}"
INSTALL_BASE="${REPOSITORY_ROOT}/install_${SUBJECT}"
LOG_BASE="${REPOSITORY_ROOT}/log_${SUBJECT}"

if [[ "${CLEAN}" == true ]]; then
  echo "清理 ${LABEL} 的旧构建目录……"
  rm -rf -- "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
fi

echo "开始构建 ${LABEL}（Release）"
echo "  源码目录：${REPOSITORY_ROOT}/src"
echo "  构建目录：${BUILD_BASE}"
echo "  安装目录：${INSTALL_BASE}"
echo "  日志目录：${LOG_BASE}"
echo "  生产包：${PACKAGES[*]}"

cd "${REPOSITORY_ROOT}"
colcon \
  --log-base "${LOG_BASE}" \
  build \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --executor sequential \
  --packages-select "${PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "${LABEL} 构建完成。使用前执行："
echo "  source ${INSTALL_BASE}/setup.bash"
