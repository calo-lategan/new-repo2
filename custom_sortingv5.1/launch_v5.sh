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
#   8. ros2 launch app custom_sorting_nodev51.launch.py [args...]
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
#   JETARM_V5_DEBUG=1          pass debug=true to the node
#
# Pass any launch args as positional args, e.g.:
#   ./launch_v5.sh motion_speed:=2.0

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
# v5.1 (C2): whether THIS launch brings up depth_camera. Set false below only
# if we restart the factory service (which opens the Orbbec itself), so the
# device is never opened twice. Default true = v5 owns the camera.
V5_BRINGUP_DEPTH="true"
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
PROFILES_DIR="${JETARM_V5_PROFILES:-$HOME/jetarm_v5_profiles}"

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

stage launcher "==> JetArm Sort v5"
stage launcher "    Workspace : $WS_DIR"
stage launcher "    ROS distro: $ROS_DISTRO"
stage launcher "    Service   : $SERVICE_NAME"
stage launcher "    Profiles  : $PROFILES_DIR"
stage launcher "    Camera    : $CAMERA_TOPIC"

# -----------------------------------------------------------------------------
# Mode-picker modal (Zenity). Only shown when:
#  - SKIP_MODAL is not set to 1
#  - we have a DISPLAY (i.e. running from the desktop, not ssh-no-X)
#  - zenity is on PATH
#  - no positional args were already passed (those override the modal)
#
# Sets MODAL_ARGS based on the user's pick; merged into LAUNCH_ARGS later.
# -----------------------------------------------------------------------------
MODAL_ARGS=()
MODE=""
# Profile picker. Three tiers, in order of preference:
#   1. zenity GUI (needs DISPLAY + zenity installed)
#   2. whiptail arrow-key TUI in the terminal (preinstalled on Ubuntu/Debian)
#   3. plain numbered menu (works anywhere, even minimal busybox)
# All three default to "debug" - press ENTER at the prompt to take it.
if [ "${SKIP_MODAL:-0}" != "1" ] && [ $# -eq 0 ]; then
    if [ -n "${DISPLAY:-}" ] && command -v zenity >/dev/null 2>&1; then
        stage launcher "showing launch profile picker (zenity GUI)..."
        MODE=$(zenity --list --radiolist \
            --title="JetArm Sort v5 - pick a profile" \
            --text="Select what to launch:" \
            --width=560 --height=340 \
            --column="Pick" --column="Profile" --column="What it does" \
            FALSE "default"     "Standard: sorting node + tuner UI + rqt auto-popup" \
            TRUE  "debug"       "Default + verbose ROS log + extra stage prints" \
            FALSE "headless"    "No rqt auto-popup. Buttons in tuner UI still work." \
            FALSE "camera-off"  "Boot with camera subscription paused (debug pub path)" \
            FALSE "ai-off"      "Boot with inference paused (raw camera view only)" \
            FALSE "no-ui"       "Skip the Tkinter tuner UI (background only)" \
            2>/dev/null) || MODE="__cancel__"
    elif command -v whiptail >/dev/null 2>&1 && [ -t 0 ] && [ -t 1 ]; then
        # Arrow-key TUI menu via whiptail. Pre-installed on Ubuntu/Debian
        # (debconf dep). Renders inside the terminal - works whether or
        # not we have a DISPLAY. --default-item pre-highlights "debug".
        stage launcher "showing launch profile picker (whiptail TUI)..."
        if [ -z "${DISPLAY:-}" ]; then
            stage launcher "  (no DISPLAY - this is normal, picker still works)"
        elif ! command -v zenity >/dev/null 2>&1; then
            stage launcher "  (zenity not installed - using whiptail; install zenity for GUI)"
        fi
        # whiptail writes the chosen tag to stderr; capture via fd 3 trick
        MODE=$(whiptail \
            --title "JetArm Sort v5 - pick a profile" \
            --menu "Use arrow keys to pick a profile, then Enter:" \
            20 78 8 \
            --default-item "debug" \
            "default"    "Standard: sorting + tuner UI + rqt auto-popup" \
            "debug"      "Default + verbose ROS log + extra stage prints" \
            "headless"   "No rqt auto-popup (tuner UI buttons still work)" \
            "camera-off" "Boot with camera subscription paused" \
            "ai-off"     "Boot with inference paused (raw camera view only)" \
            "no-ui"      "Skip the Tkinter tuner UI (background only)" \
            3>&1 1>&2 2>&3) || MODE="__cancel__"
    else
        # Plain numbered text menu. Final fallback for minimal containers.
        if ! [ -t 0 ]; then
            stage launcher "stdin is not a tty - defaulting to debug (no prompt)"
            MODE="debug"
        else
            stage launcher "showing launch profile picker (numbered text menu)..."
            printf '\n\033[1;36m========== JetArm Sort v5 - pick a profile ==========\033[0m\n'
            printf '  1) default     Standard: sorting node + tuner UI + rqt auto-popup\n'
            printf '  2) debug       Default + verbose ROS log + extra stage prints  \033[1m[DEFAULT]\033[0m\n'
            printf '  3) headless    No rqt auto-popup (tuner UI buttons still work)\n'
            printf '  4) camera-off  Boot with camera subscription paused\n'
            printf '  5) ai-off      Boot with inference paused (raw camera only)\n'
            printf '  6) no-ui       Skip the Tkinter tuner UI (background only)\n'
            printf '  q) quit\n'
            printf '\033[1;36m=======================================================\033[0m\n'
            printf 'Enter choice [1-6/q, default=2 debug, 15s timeout]: '
            REPLY=""
            if ! read -r -t 15 REPLY; then
                printf '\n'
                stage launcher "no input within 15s - defaulting to debug"
                REPLY=2
            fi
            case "${REPLY:-2}" in
                1) MODE="default" ;;
                2|"") MODE="debug" ;;
                3) MODE="headless" ;;
                4) MODE="camera-off" ;;
                5) MODE="ai-off" ;;
                6) MODE="no-ui" ;;
                q|Q) MODE="__cancel__" ;;
                *) stage launcher "unrecognised choice '$REPLY' - defaulting to debug"
                   MODE="debug" ;;
            esac
        fi
    fi
    case "$MODE" in
        __cancel__|"")
            stage launcher "modal cancelled - exiting without launch"
            exit 0 ;;
        debug)
            export JETARM_V5_DEBUG=1
            MODAL_ARGS+=("debug:=true") ;;
        headless)
            MODAL_ARGS+=("image_view:=false") ;;
        camera-off)
            MODAL_ARGS+=("enable_camera_sub:=false") ;;
        ai-off)
            MODAL_ARGS+=("enable_inference:=false") ;;
        no-ui)
            MODAL_ARGS+=("tune_ui:=false") ;;
        default)
            : ;;
    esac
    stage launcher "  selected profile: $MODE (args: ${MODAL_ARGS[*]:-none})"
fi

# -----------------------------------------------------------------------------
# STEP 0 (always first): stop + disable the factory app.
# -----------------------------------------------------------------------------
# Same intent as `SKIP_SERVICE_RESTART=1` - we never want the factory
# bringup chain running alongside our launch (it grabs the camera, the
# serial port, and runs all its own pick/sort nodes that would conflict
# with v5). Idempotent: if already disabled and stopped, both commands
# no-op silently. Runs every launch so even a manual `systemctl enable`
# before reboot gets undone next time we start v5.
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

# Source third-party ROS 2 workspaces. The orbbec_camera plugin lives in
# a separate workspace; the Hiwonder factory service was previously
# sourcing it implicitly via its bringup chain. Since we now stop+disable
# that service every launch, we must source the third-party workspaces
# ourselves - otherwise the depth_cam component container fails with
# "Could not find requested resource in ament index" and cam_fps stays
# at 0 forever.
for ws_setup in \
    "${ORBBEC_WS:-/home/ubuntu/third_party_ros2/orbbec_ws}/install/setup" \
    "${THIRD_PARTY_ROS_DIR:-/home/ubuntu/third_party_ros2}/setup" \
; do
    if [ -f "${ws_setup}.bash" ] || [ -f "${ws_setup}.zsh" ]; then
        if source_setup "$ws_setup"; then
            :  # source_setup already logged the [env] line
        fi
    fi
done

# Hiwonder env vars - the start_app_node and our v4 node both expect these.
export CAMERA_TYPE="${CAMERA_TYPE:-GEMINI}"
export CHASSIS_TYPE="${CHASSIS_TYPE:-Slide_Rails}"
export need_compile="${need_compile:-False}"
export JETARM_V5_PROFILES="$PROFILES_DIR"

# ROS_DOMAIN_ID pin. ROS 2 DDS partitions topics by domain - if the
# launcher process is in domain 0 (desktop shortcut) and an interactive
# terminal has ROS_DOMAIN_ID=5 in zshrc, they can't see each other's
# topics. We pin to a known value and print it loudly so anyone running
# probes in another terminal can match it.
#
# Override with:  ROS_DOMAIN_ID=5 ~/jetarm_v5/launch_v5.sh
# Or persistently:  echo "ROS_DOMAIN_ID=5" > ~/.jetarm_v5.env  (sourced below)
if [ -f "$HOME/.jetarm_v5.env" ]; then
    # shellcheck disable=SC1091
    . "$HOME/.jetarm_v5.env"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

stage env "CAMERA_TYPE=$CAMERA_TYPE CHASSIS_TYPE=$CHASSIS_TYPE need_compile=$need_compile"
stage env "ROS_DOMAIN_ID=$ROS_DOMAIN_ID    <-- match this in other terminals!"
stage env "  to probe from another shell:"
stage env "    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
stage env "    source /opt/ros/humble/setup.bash"
stage env "    source ~/ros2_ws/install/setup.bash"

# Check whether orbbec_camera will be loadable by the depth_cam component
# container. `ros2 pkg list` is unreliable here: it only sees packages
# that registered themselves to the ament_index/packages resource, which
# the third-party orbbec_ws doesn't always do. The component container
# uses ament_index_cpp's `get_resource_lists` for "rclcpp_components",
# which is what we should actually check. Look for the resource file
# directly - if it exists, the container will find the plugin.
ORBBEC_RES="$( ls -1 /home/ubuntu/third_party_ros2/orbbec_ws/install/orbbec_camera/share/ament_index/resource_index/rclcpp_components/orbbec_camera 2>/dev/null \
              || find "${ORBBEC_WS:-/home/ubuntu/third_party_ros2/orbbec_ws}/install" \
                      -path '*/ament_index/resource_index/rclcpp_components/orbbec_camera' \
                      -print -quit 2>/dev/null )"
if [ -n "$ORBBEC_RES" ]; then
    stage env "orbbec_camera: rclcpp_components resource OK ($ORBBEC_RES)"
elif [ -f /home/ubuntu/third_party_ros2/orbbec_ws/install/orbbec_camera/lib/liborbbec_camera.so ]; then
    # Fall back to checking the library file - the resource file may live
    # under a different prefix.
    stage env "orbbec_camera: library file present (resource lookup deferred to component container)"
else
    err env "orbbec_camera: cannot find library or resource file"
    err env "  expected one of:"
    err env "    /home/ubuntu/third_party_ros2/orbbec_ws/install/orbbec_camera/lib/liborbbec_camera.so"
    err env "    \$ORBBEC_WS/install/orbbec_camera/lib/liborbbec_camera.so"
    err env "  set ORBBEC_WS=/path/to/orbbec_ws to override"
fi

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
#
# v5.1 NOTE (C2 - camera-steal): STEP 0 unconditionally `export
# SKIP_SERVICE_RESTART=1`, so the FORCE_SERVICE_RESTART branch below is
# intentionally UNREACHABLE. That is deliberate: restarting the factory
# service would run its bringup.launch.py, which opens the Orbbec device a
# SECOND time (its own 18s-delayed depth_camera) while THIS launch already
# brings up depth_camera - opening the device twice kills the camera. Do NOT
# "fix" the dead branch by removing the STEP 0 force without also stripping
# depth_camera from this launch (or gating it), or you reintroduce the steal.
# -----------------------------------------------------------------------------
if [ "$SKIP_SERVICE_RESTART" = "1" ]; then
    stage service "SKIP_SERVICE_RESTART=1 - leaving $SERVICE_NAME alone"
elif [ "$FORCE_SERVICE_RESTART" = "1" ]; then
    # v5.1 (C2): the factory bringup we're about to (re)start opens the Orbbec
    # itself, so tell our launch NOT to also bring up depth_camera - otherwise
    # the device is opened twice and the camera is stolen. This makes the two
    # paths mutually exclusive instead of relying on a "one side loses" race.
    V5_BRINGUP_DEPTH="false"
    stage service "FORCE_SERVICE_RESTART=1 - factory owns the camera; v5 skips"
    stage service "  its own depth_camera (bringup_depth_camera:=false)."
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
# STEP 6-7: pre-launch waits.
# -----------------------------------------------------------------------------
# v5 disables the factory service and starts everything (sdk, depth_cam,
# controllers, web_video_server, v4 node, tuner UI) from inside our own
# `ros2 launch` below. So waiting BEFORE the launch for the factory's
# service / controllers / camera topic always fails - they don't exist
# until our launch creates them. The waits used to add 100+ s per launch
# of pointless timeouts.
#
# Set WAIT_FOR_FACTORY_SERVICE=1 to restore the legacy pre-launch waits
# (only useful if you've left the factory service enabled - i.e. didn't
# use v5's STEP 0 disable).
WAIT_FOR_FACTORY_SERVICE="${WAIT_FOR_FACTORY_SERVICE:-0}"
if [ "$WAIT_FOR_FACTORY_SERVICE" = "1" ]; then
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
            if timeout 2 ros2 topic echo --once "$CAMERA_TOPIC" >/dev/null 2>&1; then
                ok camera "camera publishing frames (took ${i}s total)"
                cam_ok=1; break
            fi
        fi
        sleep 1
    done

    if [ "$cam_ok" -ne 1 ]; then
        err camera "camera did not start publishing within ${CAMERA_READY_TIMEOUT}s"
        err camera "we'll launch anyway - the v4 node will print [camera] errors if it can't see frames"
    fi

    # Round 15: also soft-wait for the depth topic. Not fatal - depth often
    # comes up a few seconds after RGB - but the user needs to know.
    DEPTH_TOPIC="${DEPTH_TOPIC:-/depth_cam/depth/image_raw}"
    DEPTH_WAIT="${DEPTH_WAIT:-15}"
    stage depth "soft-waiting up to ${DEPTH_WAIT}s for $DEPTH_TOPIC..."
    depth_ok=0
    for ((i=0; i<DEPTH_WAIT; i++)); do
        if ros2 topic list 2>/dev/null | grep -q "^${DEPTH_TOPIC}$"; then
            if timeout 2 ros2 topic echo --once "$DEPTH_TOPIC" >/dev/null 2>&1; then
                ok depth "depth publishing frames (took ${i}s)"
                depth_ok=1; break
            fi
        fi
        sleep 1
    done
    if [ "$depth_ok" -ne 1 ]; then
        err depth "depth topic NOT publishing yet ($DEPTH_TOPIC)"
        err depth "the Depth tab's plane fit + the depth heatmap will be unavailable"
        err depth "until it does. Check: ros2 topic list | grep depth_cam"
    fi
else
    stage launcher "skipping pre-launch service/controllers/camera waits"
    stage launcher "  (v5 brings everything up inside ros2 launch below;"
    stage launcher "   set WAIT_FOR_FACTORY_SERVICE=1 to restore legacy waits)"
fi

if [ "$EXTRA_BOOT_GRACE" -gt 0 ]; then
    stage launcher "extra ${EXTRA_BOOT_GRACE}s grace..."
    sleep "$EXTRA_BOOT_GRACE"
fi

# -----------------------------------------------------------------------------
# STEP 8: actually launch v4.
# -----------------------------------------------------------------------------
LAUNCH_ARGS=("$@")
# Append modal selections (no-op if modal was skipped or canceled to default).
if [ "${#MODAL_ARGS[@]}" -gt 0 ]; then
    LAUNCH_ARGS+=("${MODAL_ARGS[@]}")
fi
# v5.1 (C2): pass the camera-bringup interlock decided in the service step
# (true normally; false only when FORCE_SERVICE_RESTART hands the camera to the
# factory service) so this launch never double-opens the Orbbec.
LAUNCH_ARGS+=("bringup_depth_camera:=${V5_BRINGUP_DEPTH}")
if [ "${JETARM_V5_DEBUG:-0}" = "1" ]; then
    LAUNCH_ARGS+=("debug:=true")
    export JETARM_V5_DEBUG=1
fi

# Pin Jetson to max clocks for deterministic throughput.
# Without jetson_clocks, CPU stays at 729MHz and GPU at 305MHz until the
# governor boosts them - causing 2-3x inference jitter (27ms vs 61ms).
# Sudoers tip: <user> ALL=(ALL) NOPASSWD: /usr/sbin/nvpmodel, /usr/bin/jetson_clocks
stage perf "pinning Jetson to MAXN + max clocks..."
sudo -n nvpmodel -m 0 2>/dev/null \
    || sudo nvpmodel -m 0 2>/dev/null \
    || err perf "nvpmodel -m 0 failed (add sudoers rule or run manually)"
sudo -n jetson_clocks 2>/dev/null \
    || sudo jetson_clocks 2>/dev/null \
    || err perf "jetson_clocks failed (add sudoers rule or run manually)"
ok perf "clocks pinned (or skipped if sudoers not set - add rule to avoid prompt)"

stage launcher "ros2 launch app custom_sorting_nodev51.launch.py ${LAUNCH_ARGS[*]}"
ros2 launch app custom_sorting_nodev51.launch.py "${LAUNCH_ARGS[@]}"
RC=$?

if [ $RC -ne 0 ]; then
    err launcher "launch exited with code $RC"
    err launcher "see [v4][stage] lines above for the failing step"
    read -r -p "Press Enter to close..."
fi
exit $RC
