#!/usr/bin/env bash
# restart_v5.sh - cleanly restart the JetArm Sort v5.1 stack.
#
# Use this when the vendor calibration node has crashed ("vendor calibration
# node not running"), the UI died, or you just want a fresh stack before
# (re)running AprilTag calibration / lab_config.
#
# RUN IT FROM A TERMINAL IN THE DESKTOP SESSION (it relaunches GUI windows).
# Over SSH the GUI windows won't appear - use the desktop icon instead.
#
#   bash ~/jetarm_v5_src/custom_sortingv5.1/restart_v5.sh
#
set -u

echo "[restart] bringing down any running v5/v5.1 stack..."
# Kill the launch parent first (it shuts down the nodes it spawned), then the
# UI and any stragglers (drivers, vendor calibration, lab_manager).
pkill -f "custom_sorting_nodev5.*\.launch\.py" 2>/dev/null || true
pkill -f "launch_v5.sh"                        2>/dev/null || true
pkill -f "tune_uiv5"                           2>/dev/null || true
pkill -f "custom_sortingv5"                    2>/dev/null || true
pkill -f "app/lib/app/calibration"             2>/dev/null || true
pkill -f "app/lib/app/lab_manager"             2>/dev/null || true
sleep 3

# Belt-and-braces: the factory service also opens the servo serial + camera.
# Running it AND v5 at once causes "multiple access on port" (the controller
# drops the arm mid-motion). Make sure it's stopped before we relaunch.
if systemctl is-active --quiet start_app_node.service 2>/dev/null; then
  echo "[restart] stopping factory start_app_node.service (frees the servo serial)..."
  sudo -n systemctl stop start_app_node.service 2>/dev/null \
    || sudo systemctl stop start_app_node.service 2>/dev/null \
    || echo "[restart] WARN: could not stop factory service (add a sudoers rule)"
fi

# Optional: confirm nothing is still holding the controller/camera.
if pgrep -f "ros_robot_controller" >/dev/null 2>&1; then
  echo "[restart] WARN: a ros_robot_controller is still running - waiting 2s more..."
  sleep 2
fi

echo "[restart] relaunching v5.1..."
exec "$HOME/jetarm_v5_1/launch_v5.sh"
