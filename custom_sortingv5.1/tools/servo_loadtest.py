#!/usr/bin/env python3
"""
servo_loadtest.py - tell a DEAD SERVO apart from a FAULTY/high-resistance CABLE.

A bad power conductor or marginal connector passes the tiny idle + data current
(so register reads and idle voltage look fine) but collapses under the motor's
stall current the moment the servo tries to move. This commands each servo to a
far target and samples its OWN input voltage (vin) ~15x during the attempt:

  - vin SAGS hard during the move  -> power isn't getting through under load
                                      => CABLE / CONNECTOR fault (reseat/replace lead)
  - vin HOLDS but the servo doesn't move -> power fine, motor/gearbox dead
                                      => replace the SERVO

Compares servo 2 (shoulder) against servo 1 (base, known-good) on the same bus.

Raw serial, no ROS. STOP THE STACK FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/servo_loadtest.py
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


def pos(sid):
    return one(g(board.bus_servo_read_position, sid))


def load_test(sid, home):
    print("\n=== servo %d ===" % sid)
    board.bus_servo_enable_torque(sid, 1)
    time.sleep(0.2)
    board.bus_servo_set_position(0.6, [[sid, home]])
    time.sleep(1.0)
    start = pos(sid)
    print("  start pos: %s (home target was %s)" % (start, home))
    idle = [v for v in (vin(sid), vin(sid), vin(sid)) if isinstance(v, int)]
    idle_v = sum(idle) // len(idle) if idle else None
    print("  idle vin: %s mV (avg %s)" % (idle, idle_v))
    far = home - 100
    print("  commanding far move -> %d and sampling vin under load..." % far)
    board.bus_servo_set_position(0.6, [[sid, far]])
    samples = []
    t0 = time.time()
    while time.time() - t0 < 1.6:
        v = vin(sid)
        if isinstance(v, int):
            samples.append(v)
        time.sleep(0.08)
    p_after = pos(sid)
    lo = min(samples) if samples else None
    # moved = position changed from where it ACTUALLY started (not from the home
    # target - the frozen servo never reaches home, so comparing to home gives a
    # false 'moved').
    moved = (isinstance(p_after, int) and isinstance(start, int)
             and abs(p_after - start) > 15)
    print("  vin under load: %s" % samples)
    print("  -> min vin=%s mV   start=%s -> pos_after=%s   moved=%s"
          % (lo, start, p_after, moved))
    board.bus_servo_set_position(0.8, [[sid, home]])
    time.sleep(1.0)
    return {"idle": idle_v, "lo": lo, "pos_after": p_after, "start": start,
            "home": home, "moved": moved}


r1 = load_test(1, 500)
r2 = load_test(2, 520)

print("\n==== READ THE RESULT ====")


def sag(r):
    if r["idle"] is None or r["lo"] is None:
        return None
    return r["idle"] - r["lo"]


s1, s2 = sag(r1), sag(r2)
print("  servo 1 (base)     idle=%s lo=%s sag=%s mV" % (r1["idle"], r1["lo"], s1))
print("  servo 2 (shoulder) idle=%s lo=%s sag=%s mV" % (r2["idle"], r2["lo"], s2))
moved2 = bool(r2.get("moved"))
print("")
if moved2:
    print("  -> servo 2 actually MOVED under load - it's working right now.")
elif s2 is not None and s2 > 1200:
    print("  -> servo 2 voltage COLLAPSES under load (sag %s mV) while it can't move." % s2)
    print("     Power isn't getting through under current => CABLE / CONNECTOR fault.")
    print("     Reseat servo 2's lead (both ends), or swap its cable, then re-run.")
elif s2 is not None and s1 is not None and s2 > (s1 + 800):
    print("  -> servo 2 sags far more than the base under the same test => suspect its")
    print("     CABLE/CONNECTOR. Reseat/replace servo 2's lead and re-run.")
elif s2 is not None:
    print("  -> servo 2 voltage HOLDS under load (sag %s mV, ~like the base) but it" % s2)
    print("     still won't move => power delivery is fine, the MOTOR/GEARBOX is dead.")
    print("     Replace the servo (or do the physical swap test to be 100% sure).")
else:
    print("  -> couldn't sample vin cleanly; re-run, or do the physical swap test.")
print("\ndone.")
