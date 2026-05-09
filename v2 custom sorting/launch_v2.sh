#!/usr/bin/env bash
# One-click launcher for the JetArm v2 sorting stack.
#
# Mirrors the manual startup the robot needs to come up cleanly:
#   1. cd ~/ros2_ws
#   2. source install/setup.bash       (equivalent to setup.zsh from zsh)
#   3. sudo systemctl restart start_app_node.service
#   4. wait for the service to finish coming back up (the "beep")
#   5. ros2 launch app custom_sorting_nodev2.launch.py
#
# If anything fails the terminal stays open so you can read the error.
#
# Skip the systemctl step:    SKIP_SERVICE_RESTART=1 ./launch_v2.sh
# Override workspace:         WS_DIR=/path/to/ws ./launch_v2.sh

set -u

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SERVICE_NAME="${SERVICE_NAME:-start_app_node.service}"
SKIP_SERVICE_RESTART="${SKIP_SERVICE_RESTART:-0}"
SERVICE_READY_TIMEOUT="${SERVICE_READY_TIMEOUT:-30}"  # seconds
TOPIC_READY_TIMEOUT="${TOPIC_READY_TIMEOUT:-25}"      # seconds
TOPIC_READY_PATTERN="${TOPIC_READY_PATTERN:-ros_robot_controller}"
EXTRA_BOOT_GRACE="${EXTRA_BOOT_GRACE:-3}"             # seconds after topics appear

echo "==> JetArm Sort v2"
echo "    Workspace : $WS_DIR"
echo "    ROS distro: $ROS_DISTRO"
echo "    Service   : $SERVICE_NAME"

# --- 1 & 2: cd into workspace + source ROS env -----------------------------

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

# Hiwonder environment - mirror what the factory launchers set so the v2
# node sees the same camera type and chassis. Override before invoking the
# script if your hardware differs.
export CAMERA_TYPE="${CAMERA_TYPE:-GEMINI}"
export CHASSIS_TYPE="${CHASSIS_TYPE:-Slide_Rails}"
export need_compile="${need_compile:-False}"

# --- 3: restart Hiwonder's start_app_node.service --------------------------

if [ "$SKIP_SERVICE_RESTART" != "1" ]; then
    echo "==> Restarting $SERVICE_NAME (sudo)..."
    if ! sudo systemctl restart "$SERVICE_NAME"; then
        echo "ERROR: failed to restart $SERVICE_NAME" >&2
        echo "       (tip: add a sudoers rule so this never prompts:"
        echo "        '<user> ALL=(ALL) NOPASSWD: /bin/systemctl restart $SERVICE_NAME')"
        read -r -p "Press Enter to close..."
        exit 1
    fi
else
    echo "==> SKIP_SERVICE_RESTART=1 - leaving $SERVICE_NAME alone"
fi

# --- 4: wait for the service + ROS controller topics to be ready ----------
# The "beep" the robot makes when start_app_node finishes initializing isn't
# directly observable from here, so we poll: first that systemd reports the
# service active, then that the ros_robot_controller topics are visible.

echo "==> Waiting up to ${SERVICE_READY_TIMEOUT}s for $SERVICE_NAME to be active..."
for ((i=0; i<SERVICE_READY_TIMEOUT; i++)); do
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "    service is active"
        break
    fi
    sleep 1
done
if ! systemctl is-active --quiet "$SERVICE_NAME"; then
    echo "WARNING: $SERVICE_NAME did not report active in time - continuing anyway"
fi

echo "==> Waiting up to ${TOPIC_READY_TIMEOUT}s for '/${TOPIC_READY_PATTERN}*' topics..."
SAW_TOPIC=0
for ((i=0; i<TOPIC_READY_TIMEOUT; i++)); do
    if ros2 topic list 2>/dev/null | grep -q "$TOPIC_READY_PATTERN"; then
        echo "    controller topics visible"
        SAW_TOPIC=1
        break
    fi
    sleep 1
done
if [ "$SAW_TOPIC" -ne 1 ]; then
    echo "WARNING: never saw a '$TOPIC_READY_PATTERN' topic - the controller may not"
    echo "         be up yet. Launching anyway; if it fails, restart the service"
    echo "         manually and re-run."
fi

if [ "$EXTRA_BOOT_GRACE" -gt 0 ]; then
    echo "==> Extra ${EXTRA_BOOT_GRACE}s grace for the beep / full init..."
    sleep "$EXTRA_BOOT_GRACE"
fi

# --- 5: ros2 launch --------------------------------------------------------

echo "==> ros2 launch app custom_sorting_nodev2.launch.py $*"
ros2 launch app custom_sorting_nodev2.launch.py "$@"
RC=$?

if [ $RC -ne 0 ]; then
    echo
    echo "Launch exited with code $RC"
    read -r -p "Press Enter to close..."
fi
exit $RC
