#!/usr/bin/env bash
# usb_doctor.sh - one-shot diagnosis of the JetArm USB / controller-drop / power issue.
#
#   sudo bash usb_doctor.sh          # full report (run this first)
#   sudo bash usb_doctor.sh watch    # LIVE: tail USB drops while you jog the shoulder / wiggle cables
#
# Needs sudo for the kernel ring buffer (dmesg). Safe / read-only - it changes nothing.
set -o pipefail

CTRL_TTY="${CTRL_TTY:-/dev/ttyUSB0}"
sect(){ printf '\n\033[1;36m==== %s ====\033[0m\n' "$1"; }

if [ "${1:-}" = "watch" ]; then
  sect "LIVE USB WATCH (Ctrl-C to stop) - now jog the shoulder / wiggle each cable"
  exec dmesg -Tw | grep --line-buffered -iE \
    'ch341|usb .*disconnect|err = -71|cannot (reset|enable|disable)|hub_ext_port_status|over.?current|new (full|high)-speed'
fi

sect "DATE / KERNEL"; date; uname -r

sect "USB TREE"; lsusb -t
sect "USB NAMES"; lsusb

sect "WHERE IS THE SERVO CONTROLLER?"
if [ -e "$CTRL_TTY" ]; then
  printf '%s -> ' "$CTRL_TTY"
  udevadm info -q path -n "$CTRL_TTY" 2>/dev/null | sed 's#.*/usb1/##'
else
  echo "$CTRL_TTY NOT PRESENT  <-- the controller is currently dropped off the bus!"
fi
ls -l /dev/serial/by-path/ 2>/dev/null

sect "PER-DEVICE: id / speed / power / autosuspend (the 1-2 tree)"
for d in /sys/bus/usb/devices/1-2*/; do
  [ -e "$d/idVendor" ] || continue
  printf '%-11s %s:%s %4sM MaxPwr=%-6s ctrl=%-4s "%s"\n' \
    "$(basename "$d")" "$(cat "$d/idVendor")" "$(cat "$d/idProduct")" \
    "$(cat "$d/speed" 2>/dev/null)" "$(cat "$d/bMaxPower" 2>/dev/null)" \
    "$(cat "$d/power/control" 2>/dev/null)" "$(cat "$d/product" 2>/dev/null)"
done

sect "USB FAULT TALLY (which port/hub is failing - highest count = the damaged branch)"
dmesg 2>/dev/null | grep -iE 'cannot (reset|enable|disable)|cable is bad|hub_ext_port_status' \
  | grep -oE '1-2(\.[0-9]+)*(-port[0-9]+)?' | sort | uniq -c | sort -rn | head -12
echo "  ch341 disconnects total : $(dmesg 2>/dev/null | grep -c 'ch341 usb device disconnect')"
echo "  Orbbec resets total     : $(dmesg 2>/dev/null | grep -c 'onDeviceDisconnected')"
echo "  over-current events     : $(dmesg 2>/dev/null | grep -ic 'over-current')"

sect "JETSON POWER (5 samples @200ms - watch VDD_IN; a collapse = Jetson rail sagging)"
if command -v tegrastats >/dev/null 2>&1; then
  timeout 1.2 tegrastats --interval 200 2>/dev/null | sed -n '1,5p' \
    | grep -oE 'VDD_IN [0-9]+mW[^ ]*' || echo "  (no samples)"
else
  echo "  tegrastats not found"
fi

sect "CONTROLLER NODE ALIVE?"
if command -v ros2 >/dev/null 2>&1; then
  ros2 node list 2>/dev/null | grep -q ros_robot_controller \
    && echo "  ros_robot_controller: PRESENT" \
    || echo "  ros_robot_controller: ABSENT (dead, or stack not running)"
else
  echo "  ros2 not on PATH here (source the workspace if you want this check)"
fi

sect "HOW TO READ THIS"
cat <<'TXT'
- FAULT TALLY: the top line is the branch throwing the most -71/cable-bad errors
  = the physically damaged hub/cable. (We've been seeing 1-2.3 ... -port3.)
- CONTROLLER path: tells you which branch your servo board is on (e.g. 1-2.2.2).
  If the damaged branch != the controller branch -> you can unplug the damaged one.
- VDD_IN steady (no sudden drop) = the Jetson's own power is fine -> the fault is
  the arm's internal USB hub/cable, NOT the Jetson supply.
- Then run:   sudo bash usb_doctor.sh watch
  and jog the shoulder (or wiggle each cable) - the line that bursts on contact
  is the bad link.
TXT
