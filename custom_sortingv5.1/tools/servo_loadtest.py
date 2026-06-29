#!/usr/bin/env python3
"""
servo_loadtest.py - tell a DEAD SERVO apart from a FAULTY/high-resistance CABLE
by watching each servo's input voltage (vin) while it tries to move.

A bad power conductor / loose connector passes the tiny idle + data current (so
register reads and idle voltage look fine) but COLLAPSES under the motor's
current the moment the servo tries to move -> vin drops hard for a beat, the
servo browns out, and it doesn't move. A dead motor instead shows NO sag (it
draws no current). This runs the move several times and counts how often vin
collapses, so a single reading can't decide it.

  - vin COLLAPSES under load (repeatably) -> power isn't getting through under
        current => CABLE / CONNECTOR fault. Reseat/replace servo 2's lead.
  - vin HOLDS but the servo won't move -> power fine, motor/gearbox dead => servo.

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

COLLAPSE_MV = 9000  # a vin sample below this under load = power not getting through
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


def load_cycle(sid, home, far):
    """Command a far move and read vin in a tight burst to catch the inrush."""
    board.bus_servo_set_position(0.5, [[sid, far]])
    samples = []
    for _ in range(14):
        v = vin(sid)
        if isinstance(v, int):
            samples.append(v)
    board.bus_servo_set_position(0.6, [[sid, home]])
    time.sleep(0.7)
    return samples


def load_test(sid, home, cycles=4):
    print("\n=== servo %d ===" % sid)
    board.bus_servo_enable_torque(sid, 1)
    time.sleep(0.2)
    board.bus_servo_set_position(0.6, [[sid, home]])
    time.sleep(1.0)
    start = pos(sid)
    idle = [v for v in (vin(sid), vin(sid), vin(sid)) if isinstance(v, int)]
    idle_v = sum(idle) // len(idle) if idle else None
    print("  start pos=%s  idle vin avg=%s mV" % (start, idle_v))
    far = home - 100
    alls = []
    cyc_min = []
    for _ in range(cycles):
        s = load_cycle(sid, home, far)
        alls += s
        cyc_min.append(min(s) if s else None)
    p_after = pos(sid)
    lo = min(alls) if alls else None
    collapses = sum(1 for v in alls if v < COLLAPSE_MV)
    moved = (isinstance(p_after, int) and isinstance(start, int)
             and abs(p_after - start) > 15)
    print("  per-cycle min vin (mV): %s" % cyc_min)
    print("  overall min vin=%s mV   samples<%dmV: %d/%d   moved=%s"
          % (lo, COLLAPSE_MV, collapses, len(alls), moved))
    return {"idle": idle_v, "lo": lo, "collapses": collapses, "n": len(alls),
            "moved": moved, "start": start, "pos_after": p_after, "home": home}


r1 = load_test(1, 500)
r2 = load_test(2, 520)

print("\n==== READ THE RESULT ====")
print("  servo 1 (base)     idle=%s lo=%s collapses=%d/%d moved=%s"
      % (r1["idle"], r1["lo"], r1["collapses"], r1["n"], r1["moved"]))
print("  servo 2 (shoulder) idle=%s lo=%s collapses=%d/%d moved=%s"
      % (r2["idle"], r2["lo"], r2["collapses"], r2["n"], r2["moved"]))
print("")
if r2["moved"]:
    print("  -> servo 2 MOVED under load - it's working right now.")
elif r2["collapses"] >= 2:
    print("  -> servo 2 voltage COLLAPSES under load REPEATEDLY (min %s mV, %d times)."
          % (r2["lo"], r2["collapses"]))
    print("     Power isn't getting through under current => CABLE / CONNECTOR FAULT.")
    print("     Reseat BOTH ends of servo 2's lead, inspect the cable at the shoulder")
    print("     flex point, or swap the lead - then re-run. This likely SAVES the servo.")
elif r2["collapses"] == 1:
    print("  -> caught ONE voltage collapse (min %s mV) - intermittent." % r2["lo"])
    print("     Strongly suspect a loose/marginal power connection. Reseat servo 2's")
    print("     lead firmly at both ends and re-run; if the collapse returns, it's the cable.")
elif r2["lo"] is not None and r1["lo"] is not None and (r1["lo"] - r2["lo"]) > 800 and r2["lo"] < 11000:
    print("  -> servo 2 sags notably more than the base under load => suspect its lead.")
    print("     Reseat/replace servo 2's cable and re-run.")
elif r2["lo"] is not None:
    print("  -> servo 2 voltage HELD under load (min %s mV) but it still won't move" % r2["lo"])
    print("     => power delivery is fine, the MOTOR/GEARBOX is dead => replace the servo.")
else:
    print("  -> couldn't sample vin cleanly; re-run, or do the physical reseat/swap test.")
print("\ndone.")
