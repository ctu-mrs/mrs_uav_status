#!/usr/bin/env bash
# Fails if tui/, status/ or the utils/ headers they depend on pull in ANY ROS header -- message,
# service, node, clock or time -- directly or transitively. These three trees are the package's
# ROS-free layers: only uav_status.{hpp,cpp} may name an rclcpp type.
#
# Deliberately does NOT use `set -e`: grep's exit status is inspected explicitly below so
# "no forbidden includes" (1) and "grep itself failed" (>1) are never conflated.
set -uo pipefail

ROOT="$1"

# Matches #include <..._msgs/...> / "..._srvs/..." for any ROS message/service package, any
# rclcpp/* or rclcpp_*/* header (node, rclcpp, clock, time, components, ...), the low-level
# rcl/rcl_interfaces/rosidl_* and builtin_interfaces/tf2/tf2_ros headers those transitively pull
# in, plus the mrs_lib ROS-handle helpers that imply a live node/topic/service graph.
FORBIDDEN='#include[[:space:]]*[<"](([a-z_]+_msgs|[a-z_]+_srvs|rclcpp_[a-z_]+|rclcpp|rcl_interfaces|rcl|rosidl_[a-z_]+|builtin_interfaces|tf2_ros|tf2)/)|#include[[:space:]]*[<"]rclcpp\.hpp|service_client_handler\.h|subscriber_handler\.h|publisher_handler\.h|param_loader\.h'

# tui/ includes utils/ and status/ headers directly, so a forbidden dependency hiding in either
# is just as much a layering violation as one written directly under tui/. status/ (UavStatusCore)
# is scanned in its own right too -- it must stay ROS-free independent of what tui/ pulls in.
DIRS=(
  "$ROOT/include/mrs_uav_status/tui"
  "$ROOT/src/tui"
  "$ROOT/include/mrs_uav_status/utils"
  "$ROOT/include/mrs_uav_status/status"
  "$ROOT/src/status"
)
if [ -d "$ROOT/src/utils" ]; then
  DIRS+=("$ROOT/src/utils")
fi

for d in "${DIRS[@]}"; do
  if [ ! -d "$d" ]; then
    echo "ERROR: expected directory '$d' does not exist -- check_tui_ros_free.sh's scanned paths are stale (rename?)." >&2
    exit 1
  fi
done

matches=$(grep -rnE "$FORBIDDEN" "${DIRS[@]}" 2>&1)
grep_status=$?

if [ "$grep_status" -eq 0 ]; then
  echo "$matches" >&2
  echo "ERROR: tui/, status/ and the utils/ headers they depend on must not include any ROS header." >&2
  exit 1
elif [ "$grep_status" -gt 1 ]; then
  echo "$matches" >&2
  echo "ERROR: grep failed while scanning for forbidden includes (exit $grep_status)." >&2
  exit 1
fi

echo "OK: tui/, status/ and the utils/ headers they depend on have no ROS dependency."
exit 0
