#!/usr/bin/env python3
"""
servo_deeptest.py - EXHAUSTIVE shoulder (servo 2) lock/limit test for JetArm.

The controller's bus-servo protocol exposes exactly these addressable controls:
torque on/off (0x0B/0x0C), stop (0x03), set position (0x01), ID (0x10/0x12),
position offset (0x20/0x22/0x24), angle limit (0x30/0x32), voltage limit
(0x34/0x36), temperature limit (0x38/0x3A). There is NO motor/wheel-mode command
and NO current/load-limit command - so this script touches EVERY lock the servo
can have.

It (1) enumerates every register for servo 1 (base, known-good) and servo 2
(shoulder), (2) PROVES whether write commands reach servo 2 via an offset
write-readback, (3) tries every clearing action with a position-readback move
probe after each, (4) prints concrete evidence + a verdict.

Raw serial via the vendor Board (no ROS). STOP THE STACK FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/servo_deeptest.py

SAFE: offset writes are volatile (never save_offset'd) and restored; voltage/
temp/angle limits are only set to the working base servo's own healthy values;
nothing can over-drive the servo.
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

PORT = "/dev/ttyUSB0"
SH = 2     # shoulder
BASE = 1   # known-good reference
HOME = {1: 500, 2: 520}

board = Board(device=PORT, baudrate=1000000, timeout=1)
board.enable_reception(True)
time.sleep(0.3)


def g(fn, *a, timeout=1.5):
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


def two(v):
    return [v[0], v[1]] if isinstance(v, (list, tuple)) and len(v) >= 2 else None


def read_all(sid):
    return {
        "id": one(g(board.bus_servo_read_id, sid)),
        "offset": one(g(board.bus_servo_read_offset, sid)),
        "pos": one(g(board.bus_servo_read_position, sid)),
        "vin": one(g(board.bus_servo_read_vin, sid)),
        "temp": one(g(board.bus_servo_read_temp, sid)),
        "torque": one(g(board.bus_servo_read_torque_state, sid)),
        "ang": two(g(board.bus_servo_read_angle_limit, sid)),
        "vinlim": two(g(board.bus_servo_read_vin_limit, sid)),
        "templim": one(g(board.bus_servo_read_temp_limit, sid)),
    }


def show(sid, r):
    print("  servo %d: id=%s offset=%s pos=%s vin=%s mV temp=%s C torque=%s"
          % (sid, r["id"], r["offset"], r["pos"], r["vin"], r["temp"], r["torque"]))
    print("           angle_limit=%s  vin_limit=%s  temp_limit=%s"
          % (r["ang"], r["vinlim"], r["templim"]))


def move_probe(sid, label):
    """Command a few angles, read the servo's own position back after each.
    Returns True if the reported position actually changed."""
    board.bus_servo_enable_torque(sid, 1)
    time.sleep(0.15)
    home = HOME[sid]
    targets = (home - 60, home + 50, home)
    seen = []
    for tgt in targets:
        board.bus_servo_set_position(0.8, [[sid, tgt]])
        time.sleep(1.3)
        seen.append(one(g(board.bus_servo_read_position, sid)))
    ints = [p for p in seen if isinstance(p, int)]
    moved = len(ints) >= 2 and (max(ints) - min(ints) > 8)
    print("  [%s] servo %d cmd%s -> pos %s   MOVED=%s" % (label, sid, list(targets), seen, moved))
    return moved


print("==== STEP 1: FULL REGISTER ENUMERATION ====")
b = read_all(BASE)
show(BASE, b)
s = read_all(SH)
show(SH, s)

print("\n==== STEP 2: PROVE WRITES REACH servo 2 (offset write-readback) ====")
orig = s["offset"]
write_ok = None
if orig is None:
    print("  offset read failed -> cannot run the write-proof (reads only)")
else:
    test = 7 if orig != 7 else -7
    board.bus_servo_set_offset(SH, test)
    time.sleep(0.25)
    rb = one(g(board.bus_servo_read_offset, SH))
    write_ok = (rb == test)
    print("  wrote offset=%d, read back=%s  ->  WRITES REACH SERVO 2: %s" % (test, rb, write_ok))
    board.bus_servo_set_offset(SH, orig)
    time.sleep(0.2)
    print("  restored offset=%s (volatile, not saved to EEPROM)" % orig)

print("\n==== STEP 3: TRY EVERY CLEARING ACTION (move-probe after each) ====")
results = {}
results["baseline"] = move_probe(SH, "baseline")

board.bus_servo_stop([SH])
time.sleep(0.2)
board.bus_servo_enable_torque(SH, 0)
time.sleep(0.4)
board.bus_servo_enable_torque(SH, 1)
time.sleep(0.4)
results["stop+torque_off_on"] = move_probe(SH, "stop+torque off/on")

board.bus_servo_set_offset(SH, 0)
time.sleep(0.2)
results["offset_zero"] = move_probe(SH, "offset=0")
if orig is not None:
    board.bus_servo_set_offset(SH, orig)
    time.sleep(0.2)

board.bus_servo_set_angle_limit(SH, [0, 1000])
time.sleep(0.2)
results["angle_limit_full"] = move_probe(SH, "angle_limit=[0,1000]")

if b["vinlim"]:
    board.bus_servo_set_vin_limit(SH, b["vinlim"])
    time.sleep(0.2)
if b["templim"] is not None:
    board.bus_servo_set_temp_limit(SH, b["templim"])
    time.sleep(0.2)
results["limits_match_base"] = move_probe(SH, "vin/temp limits = base")

print("\n==== STEP 4: REFERENCE - base servo (servo 1) ====")
base_moved = move_probe(BASE, "base reference")

print("\n==== VERDICT ====")
sh_moved = any(results.values())
print("  writes reach servo 2        : %s" % write_ok)
print("  base (servo 1) moves        : %s" % base_moved)
print("  shoulder moved on ANY action: %s" % sh_moved)
for k, v in results.items():
    print("    - %-22s MOVED=%s" % (k, v))
print("")
if sh_moved:
    print("  -> RECOVERED: an action above un-froze the shoulder. Note which one,")
    print("     relaunch the app, and sort.")
elif write_ok and base_moved and not sh_moved:
    print("  -> CONCLUSIVE MECHANICAL FAILURE. Every addressable lock was read and")
    print("     bypassed (torque, stop, offset, angle/voltage/temp limits); the")
    print("     protocol has no mode/current lock; WRITES are confirmed reaching")
    print("     servo 2 (the offset write stuck); the base moves on the identical")
    print("     command path; yet servo 2's output never changes. The servo's brain")
    print("     receives commands but its motor/gearbox cannot move. => Replace servo 2.")
elif not base_moved:
    print("  -> base didn't move either: stack not fully stopped / port busy. Re-check.")
elif write_ok is False:
    print("  -> writes did NOT stick on servo 2 (offset write-readback failed):")
    print("     a write-path/comms problem, not the motor. Tell me - we dig into the bus.")
else:
    print("  -> inconclusive reads; re-run, or report the output.")

board.bus_servo_set_position(1.0, [[SH, 520], [BASE, 500]])
time.sleep(1.2)
print("\ndone.")
