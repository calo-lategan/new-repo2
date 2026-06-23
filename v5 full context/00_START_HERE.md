# v5 Full Context — START HERE

You are a Claude Code session being handed the **JetArm v5 custom sorting**
project. This folder is your complete briefing. Read it before touching code.

## What this project is

A pick-and-place **sorting stack for the Hiwonder JetArm** (5-DOF arm +
Orbbec depth camera + a separate USB RGB camera) running ROS 2 Humble on a
Jetson Orin Nano, inside Hiwonder's container image.

**v5** is the current/only active version. It does:
- **YOLO-only detection** (TensorRT engine, hot-swappable at runtime) — no
  OpenCV/LAB color path for detection.
- **Pick & place sorting**: detect cube/scaff → lock onto it → move arm →
  grasp (optional force-limited "compliance" grasp) → drop in a per-class
  zone.
- **Three calibration tools** in a Tkinter tuner UI, each driving the
  **vendor** ROS nodes: Position (AprilTag), Color (LAB threshold), Depth
  (table-plane fit).

## The single most important rule

**The vendor nodes are canonical. v5 is a CLIENT of them — never
reimplement them.**
- Position calibration → vendor `calibration_node` (`/calibration/*`).
- Color thresholds → vendor `lab_manager` (`/lab_manager/*`).
- Depth plane → vendor `SearchPlane` utility (called in-process).

Earlier rounds tried v5-side reimplementations (inline AprilTag solvePnP,
an in-node LAB service surface). Those were **deleted** in Round 15. If you
find yourself re-adding AprilTag/LAB math into v5, stop — wire the vendor
service instead. There is **no vendor GUI in the source** to copy; it was a
web app shipped in Hiwonder's image (see `03_ARCHITECTURE.md`). Parity =
drive the same backend services + reproduce the previews via ROS image
subscription.

## Current goal

Get **reliable end-to-end pick+place** plus the **three calibration tools**
fully working on the hardware. Most fixes are shipped but **unverified on
the arm** (this was a remote session with no device access). See
`04_HISTORY_AND_STATE.md` and `06_PLANS_DONE_AND_UNFINISHED.md`.

## Read next, in order

1. **`01_CODE_MAP.md`** — where everything lives, how the two big files are
   structured, the data flow.
2. **`02_REFERENCE_INDEX.md`** — every repo/branch/device-path/topic/
   service/config/log you can use as a reference.
3. **`03_ARCHITECTURE.md`** — how the system actually works + settled
   decisions.
4. **`04_HISTORY_AND_STATE.md`** — what changed each round + what's
   working vs unproven.
5. **`05_HOW_TO_HELP.md`** — update/test/log/git workflow for this project.
6. **`06_PLANS_DONE_AND_UNFINISHED.md`** — the full roadmap with DONE /
   PARTIAL / UNFINISHED labels + a memory-dump appendix.
7. **`INSTALL.md`**, **`UPDATE_JETARM.md`** — device install/update (copies;
   the authoritative versions are at the repo root + `custom_sortingv5/`).

## Fastest orientation if you only have 5 minutes

- Code: `custom_sortingv5/custom_sortingv5.py` (node, 3710 L) +
  `custom_sortingv5/tune_uiv5.py` (UI, 2816 L).
- The pick brain: `sorting_loop()` → `transport_thread()` → `_do_pick()` →
  `_do_place()` in the node.
- Update the device: `git -C ~/jetarm_v5_src fetch origin && git -C
  ~/jetarm_v5_src reset --hard origin/main && ~/jetarm_v5/launch_v5.sh`.
- Logs: node writes `~/jetarm_v5/logs/v5_session_*.log`; the UI's PUSH LOGS
  button ships them to the **`jetarm-logs`** branch on GitHub.
