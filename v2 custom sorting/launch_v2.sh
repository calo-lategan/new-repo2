#!/usr/bin/env bash
# One-click launcher for the JetArm v2 sorting stack.
# Sources the ROS2 workspace, exports the env vars Hiwonder's stack expects,
# and runs the v2 launch file. Intended to be invoked by the desktop shortcut
# (jetarm-sort-v2.desktop) but works standalone too.
#
# If something fails the terminal stays open so you can read the error.

set -u

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"

echo "==> JetArm Sort v2"
echo "    Workspace: $WS_DIR"
echo "    ROS distro: $ROS_DISTRO"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
    read -r -p "Press Enter to close..."
    exit 1
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
else
    echo "WARNING: ${WS_DIR}/install/setup.bash not found - did you colcon build?"
fi

# Hiwonder environment - mirror what the factory launchers set so the v2
# node sees the same camera type and chassis. Override before invoking the
# script if your hardware differs.
export CAMERA_TYPE="${CAMERA_TYPE:-GEMINI}"
export CHASSIS_TYPE="${CHASSIS_TYPE:-Slide_Rails}"
export need_compile="${need_compile:-False}"

ros2 launch app custom_sorting_nodev2.launch.py "$@"
RC=$?

if [ $RC -ne 0 ]; then
    echo
    echo "Launch exited with code $RC"
    read -r -p "Press Enter to close..."
fi
exit $RC
