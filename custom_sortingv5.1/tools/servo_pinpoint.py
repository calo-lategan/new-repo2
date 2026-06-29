#!/usr/bin/env python3
"""
servo_pinpoint.py - localize the shoulder (servo 2) power-collapse fault.

Is the bad joint SHARED (the lead feeding the shoulder / upstream - which would
starve the elbow+wrist+gripper too), or ISOLATED to the shoulder (its own input
connector or internal tap)?

It reads every joint's idle voltage, then commands ONLY the shoulder to pull
current and samples the DOWNSTREAM joints' voltage during it:
  - downstream voltage ALSO collapses -> the fault is in the SHARED path (the
    lead into the shoulder / its input connector) -> replace/reseat THAT lead.
  - downstream voltage HOLDS while the shoulder collapses -> the fault is
    ISOLATED to the shoulder -> its lead or its internal tap (swap the lead to
    decide).

Only the shoulder is moved (within the range already proven safe), so the rest
of the arm stays put.

Raw serial, no ROS. STOP THE STACK FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/servo_pinpoint.py
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

COLLAPSE_MV = 9000
SH = 2
DOWNSTREAM = [3, 4, 5]   # joints after the shoulder in the chain
UPSTREAM = 1             # base, before the shoulder

board = Board(device="/dev/ttyUSB0", baudrate=1000000, timeout=1)
board.enable_reception(True)
time.sleep(0.3)


def g(fn, *a, timeout=1.0):
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


def vin(sid):
    return one(g(board.bus_servo_read_vin, sid))


print("==== idle voltage (no load) ====")
for sid in [UPSTREAM, SH] + DOWNSTREAM:
    print("  servo %d: vin=%s mV" % (sid, vin(sid)))

print("\n==== shoulder (servo 2) pulling current - sampling EVERY joint's vin ====")
board.bus_servo_enable_torque(SH, 1)
time.sleep(0.2)
samples = {sid: [] for sid in [UPSTREAM, SH] + DOWNSTREAM}
for cyc in range(3):
    board.bus_servo_set_position(0.4, [[SH, 460]])   # shoulder draws current (safe range)
    for _ in range(5):
        for sid in [UPSTREAM, SH] + DOWNSTREAM:
            v = vin(sid)
            if isinstance(v, int):
                samples[sid].append(v)
    board.bus_servo_set_position(0.6, [[SH, 520]])
    time.sleep(0.6)

print("  (while the shoulder was drawing current)")
mins = {}
for sid in [UPSTREAM, SH] + DOWNSTREAM:
    lo = min(samples[sid]) if samples[sid] else None
    coll = sum(1 for v in samples[sid] if v < COLLAPSE_MV)
    mins[sid] = lo
    tag = "  <- SHOULDER" if sid == SH else ("  (upstream)" if sid == UPSTREAM else "  (downstream)")
    print("  servo %d: min vin=%s mV   collapses=%d/%d%s"
          % (sid, lo, coll, len(samples[sid]), tag))

board.bus_servo_set_position(1.0, [[SH, 520]])
time.sleep(0.5)

print("\n==== VERDICT ====")
down_lo = [mins[d] for d in DOWNSTREAM if isinstance(mins[d], int)]
down_collapsed = any(v < COLLAPSE_MV for v in down_lo)
sh_collapsed = isinstance(mins[SH], int) and mins[SH] < COLLAPSE_MV
if not sh_collapsed:
    print("  shoulder did NOT collapse this run - re-run (the fault is intermittent).")
elif down_collapsed:
    print("  Downstream joints' voltage ALSO collapsed while only the shoulder pulled")
    print("  current => the bad joint is in the SHARED path: the lead feeding the")
    print("  shoulder (base->shoulder) or the shoulder's INPUT connector.")
    print("  => Replace/reseat THAT lead. (The elbow/wrist would brown out under load too.)")
else:
    print("  Downstream joints HELD voltage (only the shoulder collapsed) => the fault is")
    print("  ISOLATED to the shoulder: its own lead, its input connector, or its internal")
    print("  tap. Swap the shoulder's lead with a known-good one: collapse clears => it's")
    print("  the lead (replace it); collapse stays => it's inside the servo (replace servo).")
print("\ndone.")
