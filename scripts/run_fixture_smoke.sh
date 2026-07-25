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
  echo "Build the complete development workspace before running fixture tests." >&2
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

VERIFY_MODE="${MODE}"
EXPECTED_FAULT="stale"
EXPECT_PATH_REJECTION=false
VERIFY_TIMEOUT="15.0"
LAUNCH_ARGUMENTS=()

case "${MODE}" in
  subject2)
    LAUNCH_ARGUMENTS+=("path_shape:=left")
    ;;
  subject2_right)
    LAUNCH_ARGUMENTS+=("path_shape:=right")
    ;;
  subject2_line)
    LAUNCH_ARGUMENTS+=("path_shape:=line")
    ;;
  subject2_path_timeout)
    LAUNCH_ARGUMENTS+=("path_shape:=left" "path_stop_after_s:=3.0")
    VERIFY_TIMEOUT="12.0"
    ;;
  subject2_path_replay)
    LAUNCH_ARGUMENTS+=(
      "path_shape:=left"
      "path_stamp_mode:=freeze"
      "path_freeze_stamp_after_s:=3.0"
    )
    EXPECT_PATH_REJECTION=true
    VERIFY_TIMEOUT="12.0"
    ;;
  subject2_path_wrong_frame)
    LAUNCH_ARGUMENTS+=("path_frame:=unknown_map")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_zero_stamp)
    LAUNCH_ARGUMENTS+=("path_stamp_mode:=zero")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_negative_stamp)
    LAUNCH_ARGUMENTS+=("path_stamp_mode:=negative")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_invalid_nanosec)
    LAUNCH_ARGUMENTS+=("path_stamp_mode:=invalid_nanosec")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_empty)
    LAUNCH_ARGUMENTS+=("path_empty:=true")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_wrong_pose_frame)
    LAUNCH_ARGUMENTS+=("path_pose_frame_override:=unknown_map")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_path_nonfinite)
    LAUNCH_ARGUMENTS+=("path_inject_nonfinite_x:=true")
    EXPECT_PATH_REJECTION=true
    ;;
  subject2_odom_timeout)
    LAUNCH_ARGUMENTS+=("raw_odom_stop_after_s:=6.0")
    VERIFY_TIMEOUT="15.0"
    ;;
  subject2_odom_jump)
    EXPECTED_FAULT="translation_jump"
    LAUNCH_ARGUMENTS+=("raw_odom_inject_jump_after_s:=6.0")
    VERIFY_TIMEOUT="15.0"
    ;;
  subject2_odom_invalid_stamp)
    EXPECTED_FAULT="invalid_stamp"
    LAUNCH_ARGUMENTS+=(
      "raw_odom_topic:=/localization/odom"
      "raw_odom_frame_id:=odom"
      "raw_odom_child_frame_id:=base_link"
      "raw_odom_stamp_mode_after_s:=6.0"
      "raw_odom_stamp_mode_after:=negative"
    )
    VERIFY_TIMEOUT="15.0"
    ;;
  *)
    cat >&2 <<'EOF'
用法：
  ROS_DOMAIN_ID=<空闲编号> scripts/run_fixture_smoke.sh <模式>

模式：
  subject2
  subject2_right
  subject2_line
  subject2_path_timeout
  subject2_path_replay
  subject2_path_wrong_frame
  subject2_path_zero_stamp
  subject2_path_negative_stamp
  subject2_path_invalid_nanosec
  subject2_path_empty
  subject2_path_wrong_pose_frame
  subject2_path_nonfinite
  subject2_odom_timeout
  subject2_odom_jump
  subject2_odom_invalid_stamp
EOF
    exit 2
    ;;
esac

SNAPSHOT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ugv_subject2_fixture.XXXXXX")"
SNAPSHOT="${SNAPSHOT_DIR}/last_good_subject2_odom.json"
LAUNCH_ARGUMENTS+=("odom_snapshot_directory:=${SNAPSHOT_DIR}")
LOG_FILE="${TMPDIR:-/tmp}/ugv_${MODE}_${ROS_DOMAIN_ID}.log"

cd "${REPOSITORY_ROOT}"
setsid ros2 launch ugv_mvp_tools subject2_fixture.launch.py \
  "${LAUNCH_ARGUMENTS[@]}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup_processes() {
  # Background jobs inherit SIGINT as ignored in a non-interactive shell.
  # Send one SIGTERM to the isolated fixture process group instead.
  kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    if ! ps -eo sid=,stat= |
      awk -v session="${LAUNCH_PID}" '
        $1 == session && $2 !~ /^Z/ {found = 1}
        END {exit(found ? 0 : 1)}
      '
    then
      wait "${LAUNCH_PID}" 2>/dev/null || true
      return
    fi
    sleep 0.25
  done
  kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}

cleanup_all() {
  cleanup_processes
  rm -rf -- "${SNAPSHOT_DIR}"
}
trap cleanup_all EXIT INT TERM

sleep 0.5
python3 scripts/verify_fixture_runtime.py "${VERIFY_MODE}" \
  --expected-fault "${EXPECTED_FAULT}" \
  --snapshot "${SNAPSHOT}" \
  --timeout "${VERIFY_TIMEOUT}"

cleanup_processes
trap - EXIT INT TERM
rm -rf -- "${SNAPSHOT_DIR}"

# Container PID 1 may reap exited grandchildren after this script returns.
# A zombie has no executable process left; only non-zombie session members
# indicate that the fixture failed to shut down.
SURVIVORS="$(
  ps -eo sid=,stat=,pid=,args= |
    awk -v session="${LAUNCH_PID}" '
      $1 == session && $2 !~ /^Z/ {print}
    '
)"
if [[ -n "${SURVIVORS}" ]]; then
  echo "Live fixture processes survived shutdown:" >&2
  echo "${SURVIVORS}" >&2
  exit 1
fi
if grep -qE "Traceback|process has died|Caught exception" "${LOG_FILE}"; then
  echo "Fixture launch logged an unexpected process failure:" >&2
  cat "${LOG_FILE}" >&2
  exit 1
fi
if [[ "${EXPECT_PATH_REJECTION}" == true ]] &&
  ! grep -q "Rejected /subject2/path" "${LOG_FILE}"; then
  echo "Fixture did not observe the expected controller path rejection:" >&2
  cat "${LOG_FILE}" >&2
  exit 1
fi

echo "SUBJECT2_FIXTURE_SMOKE_OK mode=${MODE} domain=${ROS_DOMAIN_ID} log=${LOG_FILE}"
