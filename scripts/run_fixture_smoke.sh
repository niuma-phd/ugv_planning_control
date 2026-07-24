#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble" >&2
  exit 2
fi

# ROS setup scripts are not safe under nounset.
source /opt/ros/humble/setup.bash
if [[ ! -f "${REPOSITORY_ROOT}/install/setup.bash" ]]; then
  echo "Build the repository before running fixture smoke tests." >&2
  exit 2
fi
source "${REPOSITORY_ROOT}/install/setup.bash"

set -euo pipefail

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo "Set an unused ROS_DOMAIN_ID explicitly." >&2
  exit 2
fi
if [[ ! "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] ||
  ((ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232)); then
  echo "ROS_DOMAIN_ID must be an integer from 0 through 232." >&2
  exit 2
fi

MODE="${1:-}"
if [[ "$#" -ne 1 ]]; then
  echo "Exactly one fixture mode is required." >&2
  exit 2
fi
LAUNCH_FILE=""
VERIFY_MODE=""
EXPECTED_FAULT="stale"
VERIFY_TIMEOUT="15.0"
LAUNCH_ARGUMENTS=()

case "${MODE}" in
  subject2)
    LAUNCH_FILE="subject2_fixture.launch.py"
    VERIFY_MODE="subject2"
    ;;
  subject2_fault)
    LAUNCH_FILE="subject2_fixture.launch.py"
    VERIFY_MODE="subject2_fault"
    LAUNCH_ARGUMENTS+=("raw_odom_stop_after_s:=6.0")
    ;;
  subject2_jump)
    LAUNCH_FILE="subject2_fixture.launch.py"
    VERIFY_MODE="subject2_fault"
    EXPECTED_FAULT="translation_jump"
    LAUNCH_ARGUMENTS+=("raw_odom_inject_jump_after_s:=6.0")
    ;;
  subject1)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1"
    LAUNCH_ARGUMENTS+=("pointcloud_scenario:=right")
    ;;
  subject1_none)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1_none"
    LAUNCH_ARGUMENTS+=("pointcloud_scenario:=none")
    ;;
  subject1_blocked)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1_blocked"
    LAUNCH_ARGUMENTS+=("pointcloud_scenario:=blocked")
    ;;
  subject1_fault)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1_fault"
    LAUNCH_ARGUMENTS+=("pointcloud_scenario:=right" "pointcloud_stop_after_s:=6.0")
    ;;
  subject1_replay)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1_replay"
    LAUNCH_ARGUMENTS+=(
      "pointcloud_scenario:=right"
      "pointcloud_freeze_stamp_after_s:=6.0"
    )
    ;;
  subject1_invalid)
    LAUNCH_FILE="subject1_fixture.launch.py"
    VERIFY_MODE="subject1_invalid"
    LAUNCH_ARGUMENTS+=("pointcloud_scenario:=all_nan")
    ;;
  *)
    echo "Usage: ROS_DOMAIN_ID=<unused> $0 {subject2|subject2_fault|subject2_jump|subject1|subject1_none|subject1_blocked|subject1_fault|subject1_replay|subject1_invalid}" >&2
    exit 2
    ;;
esac

SNAPSHOT="/home/sunrise/.ros/ugv_mvp/last_good_subject2_odom.json"
if [[ "${VERIFY_MODE}" == "subject2_fault" ]]; then
  rm -f -- "${SNAPSHOT}" "${SNAPSHOT%.json}.csv"
fi

LOG_FILE="${TMPDIR:-/tmp}/ugv_${MODE}_${ROS_DOMAIN_ID}.log"
cd "${REPOSITORY_ROOT}"
setsid ros2 launch ugv_mvp_bringup "${LAUNCH_FILE}" \
  "${LAUNCH_ARGUMENTS[@]}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 60); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      wait "${LAUNCH_PID}" 2>/dev/null || true
      return
    fi
    sleep 0.25
  done
  kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 0.5
python3 scripts/verify_fixture_runtime.py "${VERIFY_MODE}" \
  --expected-fault "${EXPECTED_FAULT}" --snapshot "${SNAPSHOT}" \
  --timeout "${VERIFY_TIMEOUT}"

cleanup
trap - EXIT INT TERM

SURVIVORS="$(
  ps -eo sid=,pid=,args= |
    awk -v session="${LAUNCH_PID}" '$1 == session {print}'
)"
if [[ -n "${SURVIVORS}" ]]; then
  echo "Fixture processes survived shutdown:" >&2
  echo "${SURVIVORS}" >&2
  exit 1
fi
if grep -qE "Traceback|process has died|Caught exception" "${LOG_FILE}"; then
  echo "Fixture launch logged an unexpected process failure:" >&2
  cat "${LOG_FILE}" >&2
  exit 1
fi

echo "RDK_FIXTURE_SMOKE_OK mode=${MODE} domain=${ROS_DOMAIN_ID} log=${LOG_FILE}"
