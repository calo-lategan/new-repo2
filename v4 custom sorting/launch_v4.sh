#!/usr/bin/env bash
# One-click launcher for the JetArm v4 sorting stack.
#
# Mirrors the manual startup the robot needs to come up cleanly:
#   1. cd ~/ros2_ws
#   2. source install/setup.bash       (equivalent to setup.zsh from zsh)
#   3. sudo systemctl restart start_app_node.service
#   4. wait for the service to finish coming back up (the "beep")
#   5. ros2 launch app custom_sorting_nodev4.launch.py [profile:=fast ...]
#
# Pass any launch args as positional args, e.g.:
#   ./launch_v4.sh profile:=fast motion_speed:=2.0
#
# Env overrides:
#   SKIP_SERVICE_RESTART=1     skip step 3
#   WS_DIR=/path/to/ws         override workspace
#   JETARM_V4_PROFILES=...     override profiles dir (default ~/jetarm_v4_profiles)

set -u

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SERVICE_NAME="${SERVICE_NAME:-start_app_node.service}"
SKIP_SERVICE_RESTART="${SKIP_SERVICE_RESTART:-0}"
SERVICE_READY_TIMEOUT="${SERVICE_READY_TIMEOUT:-30}"
TOPIC_READY_TIMEOUT="${TOPIC_READY_TIMEOUT:-25}"
TOPIC_READY_PATTERN="${TOPIC_READY_PATTERN:-ros_robot_controller}"
EXTRA_BOOT_GRACE="${EXTRA_BOOT_GRACE:-3}"
PROFILES_DIR="${JETARM_V4_PROFILES:-$HOME/jetarm_v4_profiles}"

echo "==> JetArm Sort v4"
echo "    Workspace : $WS_DIR"
echo "    ROS distro: $ROS_DISTRO"
echo "    Service   : $SERVICE_NAME"
echo "    Profiles  : $PROFILES_DIR"

# --- profiles dir is created by the node, but seed it on first run ---
mkdir -p "$PROFILES_DIR"

if [ ! -d "$WS_DIR" ]; then
    echo "ERROR: workspace dir $WS_DIR not found" >&2
    read -r -p "Press Enter to close..."
    exit 1
fi
cd "$WS_DIR" || { read -r -p "Press Enter to close..."; exit 1; }

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

export CAMERA_TYPE="${CAMERA_TYPE:-GEMINI}"
export CHASSIS_TYPE="${CHASSIS_TYPE:-Slide_Rails}"
export need_compile="${need_compile:-False}"
export JETARM_V4_PROFILES="$PROFILES_DIR"

if [ "$SKIP_SERVICE_RESTART" != "1" ]; then
    echo "==> Restarting $SERVICE_NAME (sudo)..."
    if ! sudo systemctl restart "$SERVICE_NAME"; then
        echo "ERROR: failed to restart $SERVICE_NAME" >&2
        echo "       (tip: sudoers rule:"
        echo "        '<user> ALL=(ALL) NOPASSWD: /bin/systemctl restart $SERVICE_NAME')"
        read -r -p "Press Enter to close..."
        exit 1
    fi
else
    echo "==> SKIP_SERVICE_RESTART=1 - leaving $SERVICE_NAME alone"
fi

echo "==> Waiting up to ${SERVICE_READY_TIMEOUT}s for $SERVICE_NAME to be active..."
for ((i=0; i<SERVICE_READY_TIMEOUT; i++)); do
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "    service is active"; break
    fi
    sleep 1
done

echo "==> Waiting up to ${TOPIC_READY_TIMEOUT}s for '/${TOPIC_READY_PATTERN}*' topics..."
for ((i=0; i<TOPIC_READY_TIMEOUT; i++)); do
    if ros2 topic list 2>/dev/null | grep -q "$TOPIC_READY_PATTERN"; then
        echo "    controller topics visible"; break
    fi
    sleep 1
done

if [ "$EXTRA_BOOT_GRACE" -gt 0 ]; then
    echo "==> Extra ${EXTRA_BOOT_GRACE}s grace for the beep / full init..."
    sleep "$EXTRA_BOOT_GRACE"
fi

echo "==> ros2 launch app custom_sorting_nodev4.launch.py $*"
ros2 launch app custom_sorting_nodev4.launch.py "$@"
RC=$?

if [ $RC -ne 0 ]; then
    echo
    echo "Launch exited with code $RC"
    read -r -p "Press Enter to close..."
fi
exit $RC
