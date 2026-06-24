#!/usr/bin/env bash
# JetArm v4.1 diagnostics — run from a second terminal while the node is running.
# Each section is a ONE-SHOT command that prints and exits. No infinite streams.
#
# Usage:
#   bash ~/jetarm_v4_1/tools/diag.sh            # all checks
#   bash ~/jetarm_v4_1/tools/diag.sh fps         # just topic rates
#   bash ~/jetarm_v4_1/tools/diag.sh threads     # just per-thread CPU
#   bash ~/jetarm_v4_1/tools/diag.sh pyspy       # just py-spy dump
#   bash ~/jetarm_v4_1/tools/diag.sh gpu         # just tegrastats snapshot

set -o pipefail
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID

# Source ROS env if not already done
if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        . "$HOME/ros2_ws/install/setup.bash"
    fi
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[diag] ERROR: ros2 not on PATH. Source your ROS workspace first:" >&2
        echo "  source ~/ros2_ws/install/setup.bash" >&2
        exit 1
    fi
fi

hr() { printf '\n\033[1;36m──── %s ────\033[0m\n' "$*"; }
ok() { printf '\033[1;32m[ok]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }

NODE_PID=$(pgrep -f custom_sortingv4_1 2>/dev/null | head -1)

SECTION="${1:-all}"

# ── 1. True topic rates (ground truth from DDS, not our metrics) ─────────────
if [ "$SECTION" = "all" ] || [ "$SECTION" = "fps" ]; then
    hr "1. Topic rates (5s window)"
    echo "  /custom_sortingv4_1/image_result:"
    timeout 5 ros2 topic hz /custom_sortingv4_1/image_result 2>/dev/null \
        | tail -3 || warn "topic not publishing or ros2 topic hz timed out"
    echo ""
    echo "  /depth_cam/rgb/image_raw:"
    timeout 5 ros2 topic hz /depth_cam/rgb/image_raw 2>/dev/null \
        | tail -3 || warn "camera topic not publishing"
    echo ""
    echo "  Bandwidth on /custom_sortingv4_1/image_result:"
    timeout 5 ros2 topic bw /custom_sortingv4_1/image_result 2>/dev/null \
        | tail -3 || warn "cannot get bw"
    echo ""
    echo "  Topic info (QoS + subscriber count):"
    ros2 topic info /custom_sortingv4_1/image_result --verbose 2>/dev/null \
        || warn "topic info failed"
fi

# ── 2. Per-thread CPU breakdown ───────────────────────────────────────────────
if [ "$SECTION" = "all" ] || [ "$SECTION" = "threads" ]; then
    hr "2. Per-thread CPU (top -H)"
    if [ -n "$NODE_PID" ]; then
        top -bn1 -p "$NODE_PID" -H 2>/dev/null | head -25 \
            || warn "top -H failed (is the node running?)"
    else
        warn "custom_sortingv4_1 process not found - is the node running?"
    fi
    echo ""
    hr "2b. Process summary (ps)"
    ps -eo pid,ppid,pcpu,pmem,rss,comm --sort=-pcpu 2>/dev/null | head -15
fi

# ── 3. py-spy stack dump ──────────────────────────────────────────────────────
if [ "$SECTION" = "all" ] || [ "$SECTION" = "pyspy" ]; then
    hr "3. py-spy thread stack dump"
    if [ -z "$NODE_PID" ]; then
        warn "custom_sortingv4_1 not found - is the node running?"
    elif ! command -v py-spy >/dev/null 2>&1 && ! python3 -m py_spy --version >/dev/null 2>&1; then
        warn "py-spy not installed. Install with: pip install --user py-spy"
        warn "Then run:  sudo env PATH=\$PATH py-spy dump --pid $NODE_PID"
    else
        echo "  Dump 1 (PID=$NODE_PID):"
        sudo env PATH="$PATH" py-spy dump --pid "$NODE_PID" 2>/dev/null \
            || warn "py-spy dump failed (try: sudo pip install py-spy)"
        echo ""
        echo "  Sleeping 2s then dump 2 (compare to find stuck threads)..."
        sleep 2
        echo "  Dump 2:"
        sudo env PATH="$PATH" py-spy dump --pid "$NODE_PID" 2>/dev/null \
            || warn "py-spy dump 2 failed"
    fi
fi

# ── 4. GPU + thermal snapshot ─────────────────────────────────────────────────
if [ "$SECTION" = "all" ] || [ "$SECTION" = "gpu" ]; then
    hr "4. tegrastats snapshot (5s)"
    if command -v tegrastats >/dev/null 2>&1; then
        TEGLOG=$(mktemp /tmp/jetarm_teg.XXXXX.log)
        sudo tegrastats --interval 1000 --logfile "$TEGLOG" &
        TEG_PID=$!
        sleep 5
        sudo kill $TEG_PID 2>/dev/null
        wait $TEG_PID 2>/dev/null
        echo "  Last lines from tegrastats:"
        tail -5 "$TEGLOG" 2>/dev/null || warn "no tegrastats output"
        rm -f "$TEGLOG"
        echo ""
        echo "  Key fields to check:"
        echo "    GR3D_FREQ: GPU frequency (0% at 305MHz = idle/throttled)"
        echo "    CPU [N%@Fmhz]: per-core usage and frequency (want >1000MHz)"
        echo "    Temp: thermal throttle starts at ~70°C on Orin Nano"
    else
        warn "tegrastats not found (run from Jetson)"
    fi
    echo ""
    hr "4b. nvpmodel (power mode)"
    if command -v nvpmodel >/dev/null 2>&1; then
        sudo nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null || warn "nvpmodel failed"
        echo "  Mode 0 = MAXN (all cores, max TDP) — recommended for sorting"
    else
        warn "nvpmodel not found (run from Jetson)"
    fi
fi

# ── 5. Node heartbeat (grep from running log if available) ────────────────────
if [ "$SECTION" = "all" ] || [ "$SECTION" = "heartbeat" ]; then
    hr "5. Heartbeat / stage lines (5s capture)"
    if [ -n "$NODE_PID" ]; then
        echo "  Capturing stderr from PID $NODE_PID for 5s..."
        echo "  (Look for: sorting_thread=DEAD, annotated frame stale, CRASHED)"
        timeout 5 cat /proc/"$NODE_PID"/fd/2 2>/dev/null \
            | grep -E '\[v4\]\[(heartbeat|sorting-loop|publish|CRITICAL)\]' \
            || warn "cannot read process stderr (normal on some kernels)"
        echo ""
        echo "  tip: launch with debug:=true then grep in a second terminal:"
        echo "    ros2 launch app custom_sorting_nodev4.1.launch.py debug:=true 2>&1 | grep -E '\\[v4\\]\\[(heartbeat|CRASHED|sorting-loop)\\]'"
    else
        warn "node not running"
    fi
fi

hr "DONE"
echo "  Key things to look for:"
echo "    - sorting_thread=DEAD in heartbeat → thread crashed (check CRASHED log)"
echo "    - GR3D_FREQ 0%@[305] → run: sudo nvpmodel -m 0 && sudo jetson_clocks"
echo "    - topic bw flat/constant → duplicate frames (viewer frozen)"
echo "    - py-spy dump shows sorting-loop in same place twice → blocked"
