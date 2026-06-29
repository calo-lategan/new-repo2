#!/usr/bin/env python3
"""
servo_unlatch.py - deep-dive single-terminal test + latch-clear for the JetArm
shoulder (servo 2). Talks STRAIGHT to the controller over /dev/ttyUSB0 (no ROS,
no domain), so it works when the camera/ROS path can't.

What it does:
  1. FULL REGISTER DUMP of servo 1 (base, known-good) and servo 2 (shoulder):
     id / position / voltage / temperature / torque-state / angle-limit /
     voltage-limit / temperature-limit. (Every read is timeout-guarded so a
     non-answering servo can't hang the script.)
  2. DIAGNOSE servo 2: flags any protection register that would auto-trip a
     torque cut given the servo's CURRENT state (e.g. voltage outside its
     vin-limit, temperature at/over its temp-limit, position outside its
     angle-limit).
  3. CLEAR LATCH: matches servo 2's voltage/temperature protection limits to
     the working base servo's values (only if they differ), then does a
     stop + torque OFF -> ON cycle. (It does NOT change the angle-limit and
     never over-drives the servo - safe.)
  4. MOVE TEST with readback: commands servo 2 (then servo 1) to a few angles
     and reads the servo's OWN position back, so the numbers prove whether the
     joint physically moved - no camera needed.

The ROS stack owns /dev/ttyUSB0, so STOP IT FIRST:
    pkill -f custom_sorting_node ; pkill -f ros_robot_controller
    sudo systemctl stop start_app_node.service
    python3 ~/jetarm_v5_src/custom_sortingv5.1/tools/servo_unlatch.py

Verdict at the end tells you: latch cleared (shoulder moves), still frozen
(needs a hardware power-cycle), or mechanically dead (replace the servo).
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
HOME = {1: 500, 2: 520}

board = Board(device=PORT, baudrate=1000000, timeout=1)
board.enable_reception(True)
time.sleep(0.3)


def guarded(fn, *a, timeout=1.5):
    """Run a possibly-blocking SDK read in a daemon thread; None on hang/error."""
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


def first(v):
    return v[0] if isinstance(v, (list, tuple)) and len(v) >= 1 else None


def pair(v):
    return [v[0], v[1]] if isinstance(v, (list, tuple)) and len(v) >= 2 else None


def dump(sid):
    print("\n--- servo %d ---" % sid)
    out = {
        "id": guarded(board.bus_servo_read_id, sid),
        "pos": first(guarded(board.bus_servo_read_position, sid)),
        "vin": first(guarded(board.bus_servo_read_vin, sid)),
        "temp": first(guarded(board.bus_servo_read_temp, sid)),
        "torque": first(guarded(board.bus_servo_read_torque_state, sid)),
        "ang": pair(guarded(board.bus_servo_read_angle_limit, sid)),
        "vinlim": pair(guarded(board.bus_servo_read_vin_limit, sid)),
        "templim": first(guarded(board.bus_servo_read_temp_limit, sid)),
    }
    print("  id=%s pos=%s vin=%s mV temp=%s C torque_state=%s"
          % (first(out["id"]), out["pos"], out["vin"], out["temp"], out["torque"]))
    print("  angle_limit=%s  vin_limit=%s mV  temp_limit=%s C"
          % (out["ang"], out["vinlim"], out["templim"]))
    return out


print("==== FULL REGISTER DUMP ====")
s1 = dump(1)
s2 = dump(2)

print("\n==== DIAGNOSE servo 2 (shoulder) ====")
flags = []
if s2["vin"] is not None and s2["vinlim"]:
    if not (s2["vinlim"][0] <= s2["vin"] <= s2["vinlim"][1]):
        flags.append("voltage %s mV OUTSIDE vin_limit %s -> voltage protection cut"
                     % (s2["vin"], s2["vinlim"]))
if s2["temp"] is not None and s2["templim"] is not None and s2["temp"] >= s2["templim"]:
    flags.append("temp %s C >= temp_limit %s C -> over-temp protection cut"
                 % (s2["temp"], s2["templim"]))
if s2["pos"] is not None and s2["ang"] and not (s2["ang"][0] <= s2["pos"] <= s2["ang"][1]):
    flags.append("position %s OUTSIDE angle_limit %s" % (s2["pos"], s2["ang"]))
if flags:
    print("  PROTECTION REGISTERS THAT WOULD TRIP A TORQUE CUT:")
    for f in flags:
        print("   - " + f)
else:
    print("  no protection register out of range now -> if it was frozen, it was")
    print("  a firmware overload latch (only a power-cycle clears that kind).")

print("\n==== CLEAR LATCH (servo 2) ====")
# Match servo 2's voltage/temperature protection limits to the working base
# servo (servo 1) if they differ. Safe: only protection thresholds, not range.
if s1["vinlim"] and s2["vinlim"] and s1["vinlim"] != s2["vinlim"]:
    print("  servo2 vin_limit %s != base %s -> matching to base" % (s2["vinlim"], s1["vinlim"]))
    board.bus_servo_set_vin_limit(2, s1["vinlim"])
if s1["templim"] is not None and s2["templim"] is not None and s1["templim"] != s2["templim"]:
    print("  servo2 temp_limit %s != base %s -> matching to base" % (s2["templim"], s1["templim"]))
    board.bus_servo_set_temp_limit(2, s1["templim"])
if s1["ang"] and s2["ang"] and s1["ang"] != s2["ang"]:
    print("  NOTE servo2 angle_limit %s != base %s (left unchanged - report this)"
          % (s2["ang"], s1["ang"]))
print("  stop + torque OFF -> ON cycle on servo 2")
board.bus_servo_stop([2])
time.sleep(0.2)
board.bus_servo_enable_torque(2, 0)
time.sleep(0.4)
board.bus_servo_enable_torque(2, 1)
time.sleep(0.4)


def move_test(sid):
    print("\n==== MOVE TEST servo %d ====" % sid)
    home = HOME[sid]
    board.bus_servo_enable_torque(sid, 1)
    time.sleep(0.2)
    results = []
    for tgt in (home - 50, home + 40, home):
        board.bus_servo_set_position(1.0, [[sid, tgt]])
        time.sleep(1.6)
        p = first(guarded(board.bus_servo_read_position, sid))
        results.append(p)
        print("  commanded %d -> pos %s" % (tgt, p))
    return results


def moved(results):
    ps = [p for p in results if isinstance(p, int)]
    return len(ps) >= 2 and (max(ps) - min(ps) > 10)


r2 = move_test(2)
r1 = move_test(1)

print("\n==== VERDICT ====")
m2, m1 = moved(r2), moved(r1)
print("  servo 2 (shoulder) moved: %s" % m2)
print("  servo 1 (base) moved:     %s" % m1)
if m2:
    print("  -> SHOULDER WORKS (latch cleared / not latched). Relaunch the app and sort.")
elif m1 and not m2:
    print("  -> base moves, shoulder frozen: the latch did NOT clear over serial.")
    print("     Do a HARDWARE power-cycle (servo power off ~10s, back on), then re-run this.")
    print("     If it's still frozen after a power-cycle, the servo is dead -> replace it.")
else:
    print("  -> neither moved: is the ROS stack stopped? is /dev/ttyUSB0 present?")

board.bus_servo_set_position(1.0, [[2, 520], [1, 500]])
time.sleep(1.5)
print("\ndone.")
