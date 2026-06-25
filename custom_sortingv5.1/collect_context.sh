#!/usr/bin/env bash
# collect_context.sh - bundle the FULL JetArm device state into ONE tarball for
# offline analysis. Captures the things only the device has: the live config
# values the arm actually uses, the (mutated) boot profiles, the running ROS
# graph + loaded params, the USB/serial/dmesg state behind disconnects, the
# deployed source, and the latest logs.
#
#   bash ~/jetarm_v5_src/custom_sortingv5.1/collect_context.sh
#
# Produces ~/jetarm_context_<timestamp>.tar.gz - copy that ONE file to the
# analysis machine (scp / USB stick). Run it WITH THE STACK RUNNING so the ROS
# graph + live params are captured (configs/logs are captured either way).
set -u

WS="${WS:-$HOME/ros2_ws}"
PROFILES="${PROFILES:-$HOME/jetarm_v5_profiles}"
LOGDIR="${LOGDIR:-$HOME/jetarm_v5/logs}"
OUT="$HOME/jetarm_context_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

echo "[collect] 1/6 configs the arm actually uses (stepper/config + app/config)..."
for d in stepper/config app/config; do
  if [ -d "$WS/src/$d" ]; then
    mkdir -p "$OUT/config/$d"
    cp -f "$WS/src/$d"/*.yaml "$OUT/config/$d/" 2>/dev/null || true
  fi
done

echo "[collect] 2/6 boot profiles (the mutated default.yaml + presets)..."
mkdir -p "$OUT/profiles"
cp -f "$PROFILES"/*.yaml "$OUT/profiles/" 2>/dev/null || true

echo "[collect] 3/6 launch files + workspace tree + deployed app source..."
mkdir -p "$OUT/launch"
find "$WS/src" -name "*.launch.py" 2>/dev/null | while read -r f; do
  rel="${f#"$WS"/src/}"; mkdir -p "$OUT/launch/$(dirname "$rel")"; cp -f "$f" "$OUT/launch/$rel" 2>/dev/null || true
done
mkdir -p "$OUT/deployed_app_py"
cp -f "$WS"/build/app/app/*.py "$OUT/deployed_app_py/" 2>/dev/null || true
( cd "$WS/src" && ls -R > "$OUT/ws_src_tree.txt" 2>/dev/null ) || true

echo "[collect] 4/6 live ROS graph + loaded params (needs the stack running)..."
{
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash" 2>/dev/null || true
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  echo "==== ros2 node list ===="; timeout 12 ros2 node list 2>&1
  echo; echo "==== ros2 topic list -t ===="; timeout 12 ros2 topic list -t 2>&1
  echo; echo "==== ros2 service list ===="; timeout 12 ros2 service list 2>&1
  echo; echo "==== ros2 param dump /custom_sortingv5 (LIVE values) ===="; timeout 25 ros2 param dump /custom_sortingv5 2>&1
} > "$OUT/ros_graph.txt" 2>&1 || true

echo "[collect] 5/6 hardware + system (USB/serial - the fling/disconnect)..."
{
  echo "==== lsusb ===="; lsusb 2>&1
  echo; echo "==== /dev tty ===="; ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>&1
  echo; echo "==== dmesg usb/tty/serial (tail 250) ===="; dmesg 2>/dev/null | grep -iE "usb|tty|serial|disconnect|reset" | tail -n 250
  echo; echo "==== factory service ===="; systemctl status start_app_node.service --no-pager 2>&1 | head -n 25
  echo; echo "==== env ===="; env | grep -iE "CHASSIS|CAMERA|ROS_DOMAIN|ROS_DISTRO|JETARM" | sort
  echo; echo "==== uname ===="; uname -a 2>&1
  echo; echo "==== engine present? ===="; ls -l /home/ubuntu/third_party_ros2/data/*.engine 2>&1
} > "$OUT/system.txt" 2>&1 || true

echo "[collect] 6/6 latest session logs..."
mkdir -p "$OUT/logs"
ls -1t "$LOGDIR"/*.log 2>/dev/null | head -n 6 | while read -r f; do
  cp -f "$f" "$OUT/logs/" 2>/dev/null || true
done

TAR="${OUT}.tar.gz"
tar -czf "$TAR" -C "$(dirname "$OUT")" "$(basename "$OUT")" 2>/dev/null
rm -rf "$OUT"
echo
echo "[collect] DONE -> $TAR"
echo "[collect] size: $(du -h "$TAR" 2>/dev/null | cut -f1)"
echo "[collect] Copy that ONE file to the analysis machine and hand it over."
