#!/usr/bin/env python3
"""
servo_writeproof.py - robustly prove whether WRITE commands reach servo 2.

servo_deeptest.py's one-shot offset readback returned None right after the
write and reported "writes don't reach" - but None is a single timed-out READ,
not a write failure (servo 2 reads fine everywhere else). This retries the
readback several times so one transient timeout can't give a false result, and
double-checks with a second value.

Raw serial, no ROS. STOP THE STACK FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/servo_writeproof.py

The offset write is VOLATILE (never save_offset'd) and restored at the end.
"""
import os
import sys
import time
import threading

CAND = [
    "/home/ubuntu/ros2_ws/build/ros_robot_controller/ros_robot_controller",
    "/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller",
]
SDK = next((d for d in CAND if os.path.exists(d + "/ros_robot_controller_sdk.py")), None)
if SDK is None:
    sys.exit("ros_robot_controller_sdk.py not found - edit CAND")
sys.path.insert(0, SDK)
from ros_robot_controller_sdk import Board  # noqa: E402

SH = 2
board = Board(device="/dev/ttyUSB0", baudrate=1000000, timeout=1)
board.enable_reception(True)
time.sleep(0.3)


def g(fn, *a, timeout=1.2):
    box = {}

    def w():
        try:
            box["v"] = fn(*a)
        except Exception:
            box["v"] = None

    t = threading.Thread(target=w, daemon=True)
    t.start()
    t.join(timeout)
    return box.get("v", None)


def one(v):
    return v[0] if isinstance(v, (list, tuple)) and len(v) >= 1 else None


def read_offset_retry(sid, tries=8):
    for i in range(tries):
        v = one(g(board.bus_servo_read_offset, sid))
        if v is not None:
            return v, i + 1
        time.sleep(0.15)
    return None, tries


print("==== baseline: can we read servo 2's offset reliably? ====")
hits = 0
for i in range(5):
    v, _ = read_offset_retry(SH, tries=3)
    print("  read %d: offset=%s" % (i + 1, v))
    if v is not None:
        hits += 1
print("  -> reads answered %d/5" % hits)

# NOTE: offset is a BAD write-proof on this firmware - set_offset (0x20) writes a
# *temporary* deviation while read_offset (0x22) returns the *saved* value, so the
# readback never reflects the write. Use ANGLE LIMIT instead: set (0x30) and read
# (0x32) share one register, so the readback faithfully reflects the write.
def read_ang_retry(sid, tries=8):
    for i in range(tries):
        v = g(board.bus_servo_read_angle_limit, sid)
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            return [v[0], v[1]], i + 1
        time.sleep(0.15)
    return None, tries


orig_ang, _ = read_ang_retry(SH)
print("\n==== write-readback via ANGLE LIMIT (set/read share one register) ====")
print("  original angle_limit = %s" % orig_ang)
if orig_ang is None:
    sys.exit("  cannot read angle_limit - reads failing; check setup/power.")
test_lim = [10, 990] if orig_ang != [10, 990] else [20, 980]
board.bus_servo_set_angle_limit(SH, test_lim)
time.sleep(0.3)
rb, used = read_ang_retry(SH, tries=8)
print("  wrote angle_limit=%s -> read back=%s (after %d attempt(s))" % (test_lim, rb, used))
board.bus_servo_set_angle_limit(SH, orig_ang)
time.sleep(0.3)
print("  restored angle_limit -> %s" % (read_ang_retry(SH)[0]))

print("\n==== VERDICT ====")
if rb == test_lim:
    print("  WRITES REACH SERVO 2 = TRUE (angle_limit write stuck + read back).")
    print("  Servo 2 receives AND applies write commands, yet ignores position")
    print("  commands and never moves while the base does, with no voltage sag")
    print("  under load. => its controller is alive but the motor/gearbox is dead.")
    print("  Replace servo 2 (cable already ruled out by the load test).")
elif rb is None:
    print("  angle_limit readback kept timing out -> bus flaky for servo 2 now;")
    print("  power-cycle the arm and re-run.")
else:
    print("  write did not take: read back " + str(rb) + " != " + str(test_lim) + ".")
    print("  Genuine write-path anomaly - paste this output.")
print("\ndone.")
