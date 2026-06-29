#!/usr/bin/env python3
"""
power_map.py - map the JetArm SERVO-POWER rail and pinpoint where voltage is lost.

Reads, at IDLE and UNDER LOAD (shoulder pulling current):
  * the CONTROLLER BOARD's own supply voltage  (Board.get_battery - the board's
    power input, upstream of every servo), and
  * EACH servo's input voltage (vin) along the daisy chain.

So you can see exactly where the voltage FIRST collapses:
  - board supply collapses too  -> the weak point is AT/BEFORE the board input:
    the supply (battery/adapter) itself, or the lead/connector into the board,
    or the supply current-limiting. Everything downstream starves.
  - board supply HOLDS but servos collapse -> the loss is BETWEEN the board and
    the servo bus (board's servo-power output / board->first-servo connection).
  - one servo much lower than the others -> that servo's own lead.

The Jetson's rail is SEPARATE - check it with tegrastats (see notes), it has
been steady ~5-8 W the whole time.

Raw serial, no ROS. STOP THE STACK FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/power_map.py
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
SERVOS = [1, 2, 3, 4, 5]
SH = 2

board = Board(device="/dev/ttyUSB0", baudrate=1000000, timeout=1)
board.enable_reception(True)
time.sleep(0.5)


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


def board_supply(retries=15):
    """Controller board's own input voltage (mV) - the board pushes it ~50Hz."""
    for _ in range(retries):
        v = g(board.get_battery)
        if isinstance(v, int) and v > 0:
            return v
        time.sleep(0.08)
    return None


def vin(sid):
    return one(g(board.bus_servo_read_vin, sid))


print("==== IDLE (nothing moving) ====")
idle_b = board_supply()
idle_v = {s: vin(s) for s in SERVOS}
print("  controller board supply: %s mV" % idle_b)
for s in SERVOS:
    print("  servo %d input: %s mV" % (s, idle_v[s]))

print("\n==== UNDER LOAD (shoulder pulling current) ====")
board.bus_servo_enable_torque(SH, 1)
time.sleep(0.2)
load_b = []
load_v = {s: [] for s in SERVOS}
for cyc in range(3):
    board.bus_servo_set_position(0.4, [[SH, 460]])
    for _ in range(4):
        b = board_supply(retries=4)
        if isinstance(b, int):
            load_b.append(b)
        for s in SERVOS:
            v = vin(s)
            if isinstance(v, int):
                load_v[s].append(v)
    board.bus_servo_set_position(0.6, [[SH, 520]])
    time.sleep(0.6)
board.bus_servo_set_position(1.0, [[SH, 520]])
time.sleep(0.4)

lo_b = min(load_b) if load_b else None
lo_v = {s: (min(load_v[s]) if load_v[s] else None) for s in SERVOS}


def fmt(v):
    return ("%d" % v) if isinstance(v, int) else str(v)


print("\n==== POWER MAP (mV) ====")
print("  POINT                      IDLE       UNDER-LOAD")
print("  controller board supply    %-9s  %s" % (fmt(idle_b), fmt(lo_b)))
for s in SERVOS:
    tag = "  <- shoulder" if s == SH else ""
    print("  servo %d input              %-9s  %s%s" % (s, fmt(idle_v[s]), fmt(lo_v[s]), tag))

print("\n==== VERDICT ====")
b_coll = isinstance(lo_b, int) and lo_b < COLLAPSE_MV
sv_coll = any(isinstance(lo_v[s], int) and lo_v[s] < COLLAPSE_MV for s in SERVOS)
if lo_b is None and not sv_coll:
    print("  board supply unreadable and no servo collapse seen - re-run.")
elif b_coll:
    print("  The CONTROLLER BOARD's OWN supply collapsed under load (%s mV)." % fmt(lo_b))
    print("  => the weak point is AT or BEFORE the board's power input: the SUPPLY")
    print("     (battery/adapter) itself, or the lead/connector from it into the board.")
    print("     Reseat the board power connector; use a supply rated for the peak amps;")
    print("     check the supply->board lead. NOT a servo daisy cable, NOT the servo.")
elif sv_coll and isinstance(lo_b, int):
    print("  Board supply HELD (%s mV) but the servos collapsed" % fmt(lo_b))
    print("  => the loss is BETWEEN the board and the servo bus: the board's")
    print("     servo-power output stage, or the board->first-servo connection.")
    print("     Reseat the servo-bus connector at the board; inspect that header.")
elif sv_coll and lo_b is None:
    print("  Servos collapsed but board supply couldn't be read - re-run to get the")
    print("  board reading; that's what splits 'supply/lead' from 'board->servo'.")
else:
    print("  Nothing collapsed this run - intermittent; re-run a couple of times.")
print("\ndone.")
