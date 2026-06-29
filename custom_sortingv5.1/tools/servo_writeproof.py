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

orig, _ = read_offset_retry(SH)
print("\n==== write-readback (with retry) ====")
print("  original offset = %s" % orig)
if orig is None:
    sys.exit("  cannot read offset at all - reads are failing; check the setup/power.")
test = 7 if orig != 7 else -7
board.bus_servo_set_offset(SH, test)
time.sleep(0.3)
rb, used = read_offset_retry(SH, tries=8)
print("  wrote offset=%d -> read back=%s (after %d attempt(s))" % (test, rb, used))
# second confirmation with a different value
test2 = 15 if test != 15 else -15
board.bus_servo_set_offset(SH, test2)
time.sleep(0.3)
rb2, used2 = read_offset_retry(SH, tries=8)
print("  wrote offset=%d -> read back=%s (after %d attempt(s))" % (test2, rb2, used2))
board.bus_servo_set_offset(SH, orig)
time.sleep(0.2)
print("  restored offset -> %s" % (read_offset_retry(SH)[0]))

print("\n==== VERDICT ====")
if rb == test and rb2 == test2:
    print("  WRITES REACH SERVO 2 = TRUE (two distinct offset writes both stuck).")
    print("  Servo 2 receives AND acts on write commands. Yet it ignores position")
    print("  commands and never moves, while the base obeys the identical path, and")
    print("  every addressable lock is already bypassed. => CONCLUSIVE MECHANICAL")
    print("  FAILURE: the servo's controller is alive, the motor/gearbox is not.")
    print("  Replace servo 2.")
elif rb is None and rb2 is None:
    print("  offset readback still timing out after many retries -> the bus is flaky")
    print("  for servo 2 right now. Power-cycle the arm and re-run.")
else:
    print("  writes did NOT consistently take (got %s / %s). Possible write-path")
    print("  issue - paste this output." % (rb, rb2))
print("\ndone.")
