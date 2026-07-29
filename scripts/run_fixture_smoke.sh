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
VERIFY_TIMEOUT="15.0"
LAUNCH_ARGUMENTS=()
WAYPOINT_SHAPE="left"

case "${MODE}" in
  subject2)
    ;;
  subject2_right)
    WAYPOINT_SHAPE="right"
    ;;
  subject2_line)
    WAYPOINT_SHAPE="line"
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
  subject2_waypoint_file)
    WAYPOINT_SHAPE="waypoint_file"
    VERIFY_TIMEOUT="15.0"
    ;;
  subject2_recovery_success)
    LAUNCH_ARGUMENTS+=(
      "raw_odom_stop_after_s:=6.0"
      "raw_odom_linear_speed_mps:=0.25"
      "raw_odom_queued_old_samples_after_generation:=10"
      "recovery_fixture_enabled:=true"
      "recovery_scenario:=success"
    )
    WAYPOINT_SHAPE="line"
    VERIFY_TIMEOUT="20.0"
    ;;
  subject2_recovery_no_gps)
    LAUNCH_ARGUMENTS+=(
      "raw_odom_stop_after_s:=6.0"
      "raw_odom_linear_speed_mps:=0.25"
      "recovery_fixture_enabled:=true"
      "recovery_scenario:=no_gps"
    )
    WAYPOINT_SHAPE="line"
    VERIFY_TIMEOUT="33.0"
    ;;
  subject2_recovery_restart_failed)
    LAUNCH_ARGUMENTS+=(
      "raw_odom_stop_after_s:=6.0"
      "raw_odom_linear_speed_mps:=0.25"
      "recovery_fixture_enabled:=true"
      "recovery_scenario:=restart_failed"
    )
    WAYPOINT_SHAPE="line"
    VERIFY_TIMEOUT="16.0"
    ;;
  *)
    cat >&2 <<'EOF'
用法：
  ROS_DOMAIN_ID=<空闲编号> scripts/run_fixture_smoke.sh <模式>

模式：
  subject2
  subject2_right
  subject2_line
  subject2_odom_timeout
  subject2_odom_jump
  subject2_odom_invalid_stamp
  subject2_waypoint_file
  subject2_recovery_success
  subject2_recovery_no_gps
  subject2_recovery_restart_failed
EOF
    exit 2
    ;;
esac

FIXTURE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ugv_subject2_fixture.XXXXXX")"
SNAPSHOT="${FIXTURE_DIR}/last_good_subject2_odom.json"
LAUNCH_ARGUMENTS+=("odom_snapshot_directory:=${FIXTURE_DIR}")
WAYPOINT_FILE="${FIXTURE_DIR}/${WAYPOINT_SHAPE}.csv"
if [[ "${WAYPOINT_SHAPE}" == "waypoint_file" ]]; then
  cat >"${WAYPOINT_FILE}" <<'EOF'
x_m,y_m,z_m,yaw_rad
0.0,0.0,0.0,0.0
1.0,0.4,0.0,0.4
2.0,1.0,0.0,0.7
3.0,2.0,0.0,0.7
EOF
else
  python3 - "${WAYPOINT_SHAPE}" "${WAYPOINT_FILE}" <<'PY'
import csv
import math
from pathlib import Path
import sys

shape = sys.argv[1]
output = Path(sys.argv[2])
length_m = 8.0
spacing_m = 0.25
radius_m = 6.0
count = int(math.floor(length_m / spacing_m)) + 1
sign = 1.0 if shape == "left" else -1.0
with output.open("w", encoding="utf-8", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(("x_m", "y_m", "z_m", "yaw_rad"))
    for index in range(count):
        distance = min(index * spacing_m, length_m)
        if shape == "line":
            waypoint = (distance, 0.0, 0.0, 0.0)
        else:
            angle = distance / radius_m
            waypoint = (
                radius_m * math.sin(angle),
                sign * radius_m * (1.0 - math.cos(angle)),
                0.0,
                sign * angle,
            )
        writer.writerow(waypoint)
PY
fi
LAUNCH_ARGUMENTS+=("waypoint_file:=${WAYPOINT_FILE}")
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
  rm -rf -- "${FIXTURE_DIR}"
}
trap cleanup_all EXIT INT TERM

sleep 0.5
python3 scripts/verify_fixture_runtime.py "${VERIFY_MODE}" \
  --expected-fault "${EXPECTED_FAULT}" \
  --snapshot "${SNAPSHOT}" \
  --timeout "${VERIFY_TIMEOUT}"

cleanup_processes
trap - EXIT INT TERM
rm -rf -- "${FIXTURE_DIR}"

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
echo "SUBJECT2_FIXTURE_SMOKE_OK mode=${MODE} domain=${ROS_DOMAIN_ID} log=${LOG_FILE}"
