#!/usr/bin/env bash
# tools/calibration_tools.sh - open the JetArm v5 calibration tabs in a
# standalone window on the SAME ROS domain as the running app.
#
# This is the CLI equivalent of the in-app "Open in separate window"
# button. It launches the tuner UI in calibration-only mode: it builds
# ONLY the Position / Color / Depth tabs and acts as a service client of
# the already-running custom_sortingv5 + calibration + lab_manager nodes.
# No second camera is opened and no sorting node is started.
#
# Usage:
#   bash ~/jetarm_v5_src/tools/calibration_tools.sh            # Position tab
#   bash ~/jetarm_v5_src/tools/calibration_tools.sh color      # Color tab
#   bash ~/jetarm_v5_src/tools/calibration_tools.sh depth      # Depth tab
#
# Env:
#   ROS_DOMAIN_ID  defaults to 0 (must match the running app's domain)
#   NODE_NAME      target node (default custom_sortingv5)

set -o pipefail

TAB="${1:-position}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
NODE_NAME="${NODE_NAME:-custom_sortingv5}"

case "$TAB" in
  position|color|depth) ;;
  *) echo "usage: $0 [position|color|depth]" >&2; exit 1 ;;
esac

# Source the workspace if not already on PATH.
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
fi

echo "[calib-tools] opening '$TAB' tab on ROS_DOMAIN_ID=$ROS_DOMAIN_ID -> /$NODE_NAME"
exec ros2 run app tune_uiv5 --node-name "$NODE_NAME" --calib-window="$TAB"
