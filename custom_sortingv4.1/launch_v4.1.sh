#!/usr/bin/env bash
# One-click launcher for the JetArm v4 sorting stack.
#
# Built for the Hiwonder JetArm container environment.
#
# The Hiwonder image runs everything inside a container ("hiwonder" or
# similar). The container needs a moment to come up after the terminal
# attaches; if we run ROS commands too early we get bogus "command not
# found" / "no service" errors. This script therefore goes:
#
#   1. Detect we're inside the container (wait if not)
#   2. cd ~/ros2_ws
#   3. source /opt/ros/humble/setup.zsh   (or .bash on bash)
#   4. source install/setup.zsh           (or .bash)
#   5. sudo systemctl start  start_app_node.service     # idempotent;
#                                                       # only acts if inactive.
#                                                       # FORCE_SERVICE_RESTART=1
#                                                       # for the old restart.
#       - This is what BOTH brings the camera up AND reclaims it from
#         anything that's stolen it. Without it: camera dead. With it:
#         it kicks the bringup.launch.py chain which (after a built-in
#         18s TimerAction) starts the depth_cam node, which publishes
#         /depth_cam/rgb/image_raw.
#   6. Wait for `start_app_node.service` to be active.
#   7. Wait for the actual camera topic `/depth_cam/rgb/image_raw` to
#      appear (this is what bringup's TimerAction gates - so we have to
#      wait at LEAST 18s after the service comes back).
#   8. ros2 launch app custom_sorting_nodev4.1.launch.py [args...]
#
# Anything that goes wrong is printed in a [stage] format so you can
# see exactly which step failed. Terminal stays open on error.
#
# Env overrides:
#   FORCE_SERVICE_RESTART=1    kill+restart the service (old behavior).
#                              Use only when the service is in a bad state -
#                              it disturbs a running camera.
#   SKIP_SERVICE_RESTART=1     don't even check service state. Camera must
#                              already be publishing or the camera-readiness
#                              check will fail.
#   WS_DIR=/path/to/ws         override workspace (default $HOME/ros2_ws)
#   USE_ZSH=1                  re-exec under zsh and source setup.zsh
#                              (requires zsh installed; default uses bash)
#   CAMERA_READY_TIMEOUT=45    seconds to wait for the camera topic
#   JETARM_V4_DEBUG=1          pass debug=true to the node
#
# Pass any launch args as positional args, e.g.:
#   ./launch_v4.sh profile:=fast motion_speed:=2.0

# NOTE on `set -u`: we deliberately DO NOT enable nounset because Hiwonder's
# /opt/ros/humble/setup.bash and the workspace's install/setup.bash reference
# variables like AMENT_TRACE_SETUP_FILES without defaulting them. Sourcing
# them under `set -u` aborts the launcher before any [stage] line prints.
# We still want pipefail so colcon / curl failures surface.
set -o pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SERVICE_NAME="${SERVICE_NAME:-start_app_node.service}"
# Default: only `systemctl start` (idempotent, no-op if already active).
# Old SKIP_SERVICE_RESTART=1 still works but is redundant now - we never
# RESTART by default; we only start if inactive. Set FORCE_SERVICE_RESTART=1
# when you want the old kill-and-restart behavior (e.g. recovering from a
# stuck service).
SKIP_SERVICE_RESTART="1"
FORCE_SERVICE_RESTART="${FORCE_SERVICE_RESTART:-0}"
SERVICE_READY_TIMEOUT="${SERVICE_READY_TIMEOUT:-30}"
CONTROLLER_TOPIC_TIMEOUT="${CONTROLLER_TOPIC_TIMEOUT:-25}"
CAMERA_READY_TIMEOUT="${CAMERA_READY_TIMEOUT:-45}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/depth_cam/rgb/image_raw}"
EXTRA_BOOT_GRACE="${EXTRA_BOOT_GRACE:-2}"
CONTAINER_READY_TIMEOUT="${CONTAINER_READY_TIMEOUT:-30}"
# This script is bash. We must source the *.bash* setup files - sourcing
# setup.zsh from bash fails on `${(%):-%N}` and friends. We expose USE_ZSH
# as an opt-in escape hatch (e.g. if your image only ships setup.zsh) and
# it forces the script to re-exec itself under zsh.
USE_ZSH="${USE_ZSH:-0}"
PROFILES_DIR="${JETARM_V4_PROFILES:-$HOME/jetarm_v4_profiles}"

# If USE_ZSH=1 and we're currently bash, re-exec under zsh so sourcing
# setup.zsh actually works. zsh must be installed.
if [ "$USE_ZSH" = "1" ] && [ -z "${ZSH_VERSION:-}" ]; then
    if command -v zsh >/dev/null 2>&1; then
        exec zsh "$0" "$@"
    else
        echo "[launcher] USE_ZSH=1 but zsh not installed - falling back to bash" >&2
        USE_ZSH=0
    fi
fi

stage() { printf "\033[1;36m[%s]\033[0m %s\n" "$1" "$2" >&2; }
err()   { printf "\033[1;31m[%s]\033[0m %s\n" "$1" "$2" >&2; }
ok()    { printf "\033[1;32m[%s]\033[0m %s\n" "$1" "$2" >&2; }

stage launcher "==> JetArm Sort v4.1"
stage launcher "    Workspace : $WS_DIR"
stage launcher "    ROS distro: $ROS_DISTRO"
stage launcher "    Service   : $SERVICE_NAME"
stage launcher "    Profiles  : $PROFILES_DIR"
stage launcher "    Camera    : $CAMERA_TOPIC"

# -----------------------------------------------------------------------------
# STEP 0 (always first): stop + disable the factory app.
# -----------------------------------------------------------------------------
# Same intent as `SKIP_SERVICE_RESTART=1` - we never want the factory
# bringup chain running alongside our launch (it grabs the camera, the
# serial port, and runs all its own pick/sort nodes that would conflict
# with v4.1). Idempotent: if already disabled and stopped, both commands
# no-op silently. Runs every launch so even a manual `systemctl enable`
# before reboot gets undone next time we start v4.1.
FACTORY_SERVICE="${FACTORY_SERVICE:-start_app_node.service}"
if systemctl is-active --quiet "$FACTORY_SERVICE" 2>/dev/null; then
    stage service "stopping factory service ($FACTORY_SERVICE)"
    sudo -n systemctl stop "$FACTORY_SERVICE" 2>/dev/null \
        || sudo systemctl stop "$FACTORY_SERVICE" 2>/dev/null \
        || err service "failed to stop $FACTORY_SERVICE (continuing - add sudoers rule)"
fi
if systemctl is-enabled --quiet "$FACTORY_SERVICE" 2>/dev/null; then
    stage service "disabling factory service ($FACTORY_SERVICE) - won't autostart on boot"
    sudo -n systemctl disable "$FACTORY_SERVICE" 2>/dev/null \
        || sudo systemctl disable "$FACTORY_SERVICE" 2>/dev/null \
        || err service "failed to disable $FACTORY_SERVICE (continuing - add sudoers rule)"
fi
# After this point, the rest of the script must not touch the service.
# Force SKIP_SERVICE_RESTART=1 explicitly so the legacy restart/start
# block lower down is bypassed regardless of caller env.
export SKIP_SERVICE_RESTART=1

# -----------------------------------------------------------------------------
# STEP 1: Wait for the Hiwonder container to be ready.
# -----------------------------------------------------------------------------
# The desktop shortcut opens a terminal that drops you inside the container,
# but it may not be fully initialized for a beat. We probe for two markers
# that mean "the container is alive enough to run ROS":
#   - the ROS distro setup file exists
#   - the workspace dir exists
#   - the `ros2` binary is callable
# If those aren't true immediately, we wait and retry rather than failing
# outright.

stage container "waiting up to ${CONTAINER_READY_TIMEOUT}s for container to be ready..."
container_ready=0
for ((i=0; i<CONTAINER_READY_TIMEOUT; i++)); do
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] \
       && [ -d "$WS_DIR" ]; then
        container_ready=1
        ok container "container ready (took ${i}s)"
        break
    fi
    sleep 1
done
if [ "$container_ready" -ne 1 ]; then
    err container "container never came up. Check that:"
    err container "  - you ARE inside the hiwonder container (not the host)"
    err container "  - /opt/ros/${ROS_DISTRO}/setup.bash exists"
    err container "  - $WS_DIR exists"
    read -r -p "Press Enter to close..."
    exit 1
fi

mkdir -p "$PROFILES_DIR"

# -----------------------------------------------------------------------------
# STEP 1+2: cd into workspace.
# -----------------------------------------------------------------------------
cd "$WS_DIR" || { err launcher "cd $WS_DIR failed"; read -r -p "Press Enter..."; exit 1; }
stage env "cwd = $(pwd)"

# -----------------------------------------------------------------------------
# STEP 3+4: source the ROS env. Prefer setup.zsh per Hiwonder's docs, but
# fall back to .bash so we work in either shell. Source via `.` so it
# affects our current shell context.
# -----------------------------------------------------------------------------
source_setup() {
    # Pick the right setup file for the shell we are CURRENTLY running in.
    # Sourcing a zsh-syntax file from bash blows up on ${(%):-%N} - we have
    # to match the file extension to the running shell, not the user's
    # preferred login shell.
    local base="$1"   # path WITHOUT extension
    local picked=""
    if [ -n "${ZSH_VERSION:-}" ] && [ -f "${base}.zsh" ]; then
        picked="${base}.zsh"
    elif [ -f "${base}.bash" ]; then
        picked="${base}.bash"
    elif [ -f "${base}.sh" ]; then
        picked="${base}.sh"
    fi
    if [ -n "$picked" ]; then
        # The ROS setup scripts reference variables they don't define
        # (AMENT_TRACE_SETUP_FILES, COLCON_TRACE, ...). Defensively disable
        # nounset for the duration of the source even though we already
        # don't set it globally - belt and braces.
        local prev_u; case $- in *u*) prev_u=1 ;; *) prev_u=0 ;; esac
        set +u
        # shellcheck disable=SC1090
        . "$picked"
        [ "$prev_u" = "1" ] && set -u
        stage env "sourced $picked"; return 0
    fi
    err env "no compatible setup file found under $base (have you run colcon build?)"; return 1
}

if ! source_setup "/opt/ros/${ROS_DISTRO}/setup"; then
    err env "/opt/ros/${ROS_DISTRO}/setup.{zsh,bash} not present"
    read -r -p "Press Enter to close..."
    exit 1
fi
if ! source_setup "${WS_DIR}/install/setup"; then
    err env "workspace install/setup.* missing - did you run colcon build?"
    err env "run:  cd $WS_DIR && colcon build --packages-select app && exit"
    read -r -p "Press Enter to close..."
    exit 1
fi

# Hiwonder env vars - the start_app_node and our v4 node both expect these.
export CAMERA_TYPE="${CAMERA_TYPE:-GEMINI}"
export CHASSIS_TYPE="${CHASSIS_TYPE:-Slide_Rails}"
export need_compile="${need_compile:-False}"
export JETARM_V4_PROFILES="$PROFILES_DIR"
stage env "CAMERA_TYPE=$CAMERA_TYPE CHASSIS_TYPE=$CHASSIS_TYPE need_compile=$need_compile"

# Sanity: ros2 must be callable now.
if ! command -v ros2 >/dev/null 2>&1; then
    err env "'ros2' not on PATH after sourcing setup - something is very wrong"
    read -r -p "Press Enter to close..."
    exit 1
fi

# -----------------------------------------------------------------------------
# STEP 5: restart start_app_node.service.
# This is non-negotiable for the camera:
#   - Without the service running, /depth_cam/rgb/image_raw is gone.
#   - HOWEVER: `systemctl restart` while the service is healthy is itself
#     disruptive (it kills the running depth_cam node and the 18s
#     TimerAction has to play out again). So we default to `systemctl
#     start` which is idempotent - it does NOTHING if the service is
#     already active. That way an active, working camera is never
#     disturbed.
#   - FORCE_SERVICE_RESTART=1 brings back the kill-and-restart behavior
#     (use when the service is in a bad state).
#   - SKIP_SERVICE_RESTART=1 honored for back-compat: skip the whole
#     thing. If the service is inactive anyway you'll get a loud warning
#     and the launch will fail-fast at the camera-readiness check.
# -----------------------------------------------------------------------------
if [ "$SKIP_SERVICE_RESTART" = "1" ]; then
    stage service "SKIP_SERVICE_RESTART=1 - leaving $SERVICE_NAME alone"
elif [ "$FORCE_SERVICE_RESTART" = "1" ]; then
    stage service "FORCE_SERVICE_RESTART=1 - restarting $SERVICE_NAME (sudo)..."
    if ! sudo systemctl restart "$SERVICE_NAME"; then
        err service "failed to restart $SERVICE_NAME"
        err service "sudoers tip:"
        err service "  <user> ALL=(ALL) NOPASSWD: /bin/systemctl restart $SERVICE_NAME"
        read -r -p "Press Enter to close..."
        exit 1
    fi
    ok service "restart issued"
else
    # Default: idempotent start - only acts when service is inactive.
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        ok service "$SERVICE_NAME already active - not disturbing it"
    else
        stage service "$SERVICE_NAME not active - starting (sudo)..."
        if ! sudo systemctl start "$SERVICE_NAME"; then
            err service "failed to start $SERVICE_NAME"
            err service "sudoers tip:"
            err service "  <user> ALL=(ALL) NOPASSWD: /bin/systemctl restart $SERVICE_NAME, /bin/systemctl start $SERVICE_NAME"
            read -r -p "Press Enter to close..."
            exit 1
        fi
        ok service "start issued"
    fi
fi

# -----------------------------------------------------------------------------
# STEP 6: wait for systemd to mark the service active.
# -----------------------------------------------------------------------------
stage service "waiting up to ${SERVICE_READY_TIMEOUT}s for systemd active..."
svc_ok=0
for ((i=0; i<SERVICE_READY_TIMEOUT; i++)); do
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        ok service "active (took ${i}s)"; svc_ok=1; break
    fi
    sleep 1
done
if [ "$svc_ok" -ne 1 ]; then
    err service "$SERVICE_NAME never reported active"
    err service "diagnose with: sudo journalctl -u $SERVICE_NAME -n 60 --no-pager"
fi

# Controller topics (servo / kinematics) come up first - confirm they
# exist before we wait for the camera (which is gated by a TimerAction
# inside Hiwonder's bringup.launch.py).
stage controllers "waiting up to ${CONTROLLER_TOPIC_TIMEOUT}s for /ros_robot_controller*..."
ctrl_ok=0
for ((i=0; i<CONTROLLER_TOPIC_TIMEOUT; i++)); do
    if ros2 topic list 2>/dev/null | grep -q "ros_robot_controller"; then
        ok controllers "controller topics visible (took ${i}s)"
        ctrl_ok=1; break
    fi
    sleep 1
done
if [ "$ctrl_ok" -ne 1 ]; then
    err controllers "controller topics never appeared - kinematics will hang"
    err controllers "check: ros2 topic list  /  sudo journalctl -u $SERVICE_NAME"
fi

# -----------------------------------------------------------------------------
# STEP 7: wait for the camera topic specifically.
# bringup.launch.py runs:    TimerAction(period=18.0, actions=[depth_camera_launch])
# So the camera will appear no earlier than ~18s after the service starts.
# We give it CAMERA_READY_TIMEOUT (45s by default) and we also poll for
# actual messages on the topic, not just its presence, because the topic
# can be advertised before the driver is publishing.
# -----------------------------------------------------------------------------
stage camera "waiting up to ${CAMERA_READY_TIMEOUT}s for $CAMERA_TOPIC to publish..."
cam_ok=0
cam_advertised=0
for ((i=0; i<CAMERA_READY_TIMEOUT; i++)); do
    if [ "$cam_advertised" -ne 1 ]; then
        if ros2 topic list 2>/dev/null | grep -q "^${CAMERA_TOPIC}$"; then
            cam_advertised=1
            ok camera "topic advertised (took ${i}s) - now waiting for frames"
        fi
    else
        # ros2 topic hz blocks; use a short timeout to probe.
        if timeout 2 ros2 topic echo --once "$CAMERA_TOPIC" >/dev/null 2>&1; then
            ok camera "camera publishing frames (took ${i}s total)"
            cam_ok=1; break
        fi
    fi
    sleep 1
done

if [ "$cam_ok" -ne 1 ]; then
    err camera "camera did not start publishing within ${CAMERA_READY_TIMEOUT}s"
    err camera "possible causes:"
    err camera "  - depth_cam process crashed (check journalctl)"
    err camera "  - USB unplugged / wrong CAMERA_TYPE env"
    err camera "  - another node is holding the camera open"
    err camera "we'll launch anyway - the v4 node will print [camera] errors if it can't see frames"
fi

if [ "$EXTRA_BOOT_GRACE" -gt 0 ]; then
    stage launcher "extra ${EXTRA_BOOT_GRACE}s grace..."
    sleep "$EXTRA_BOOT_GRACE"
fi

# -----------------------------------------------------------------------------
# STEP 8: actually launch v4.
# -----------------------------------------------------------------------------
LAUNCH_ARGS=("$@")
if [ "${JETARM_V4_DEBUG:-0}" = "1" ]; then
    LAUNCH_ARGS+=("debug:=true")
    export JETARM_V4_DEBUG=1
fi

stage launcher "ros2 launch app custom_sorting_nodev4.1.launch.py ${LAUNCH_ARGS[*]}"
ros2 launch app custom_sorting_nodev4.1.launch.py "${LAUNCH_ARGS[@]}"
RC=$?

if [ $RC -ne 0 ]; then
    err launcher "launch exited with code $RC"
    err launcher "see [v4][stage] lines above for the failing step"
    read -r -p "Press Enter to close..."
fi
exit $RC
