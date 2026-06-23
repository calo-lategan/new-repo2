# 03 — Architecture & Settled Decisions

## The vendor-delegation model

v5 does NOT reimplement calibration. It is a **service client** of the
vendor ROS nodes, which the v5 launch file brings up alongside the sorting
node on the same domain:

- **Position calibration** → vendor `calibration_node`. v5's
  `run_calibration` service drives `/calibration/enter → /start → wait
  /calibration/finish (Bool) → /exit`, then reloads `transform.yaml` via
  `get_roi()`.
- **Color thresholds** → vendor `lab_manager`. The Color tab calls
  `/lab_manager/{enter,get_range,change_range,stash_range,save_to_disk}`.
- **Depth plane** → vendor `SearchPlane` utility, called in-process by
  `_fit_table_plane()` and the `depth_plane_refit` service.

### Why — and the key finding about the GUI

A full-tree search found **no GUI source anywhere** in the repo (no html/js/
qml/ui/tkinter/Qt for calibration). The Hiwonder calibration & lab "tools"
are a **web frontend served over `rosbridge_server` + `web_video_server`**
(see `bringup/launch/bringup.launch.py`), shipped inside Hiwonder's app
image — NOT in this source dump. So "one-for-one with the vendor tool" can
only mean **functional parity**: drive the same backend services and
reproduce the tool's live video panels by subscribing to the same ROS image
topics (`/calibration/image_result`, `/lab_manager/image_result`). That is
exactly what v5's Position/Color tabs do (Round 16 `_make_image_preview`).

**Guardrail for you:** if a task tempts you to re-add AprilTag solvePnP or
LAB thresholding *into* v5, don't. Wire the vendor service. The inline
reimplementations were deleted in Round 15 on purpose.

## The three calibration tabs (each independent)

| Tab | Drives | Writes | Button |
|---|---|---|---|
| Position | `/calibration/*` | `transform.yaml` (extrinsic, corners) | CALIBRATE POSITION |
| Color | `/lab_manager/*` | `lab_config.yaml` (LAB ranges) | ENTER → adjust → STASH → SAVE |
| Depth | `SearchPlane` via `depth_plane_refit` | `transform.yaml` `plane:` key | CALIBRATE DEPTH (plane refit) |

Each button touches ONLY its own YAML. All three are reachable in the main
tuner AND can be popped out into a **separate window on the same ROS domain**
("Open in separate window" button, or `tools/calibration_tools.sh`).

## Depth pipeline

1. `_lookup_depth_color_tf()` resolves the static `depth_cam_link →
   depth_cam_color_frame` TF at startup and folds it into the `hand2cam`
   matrix (mirrors vendor).
2. `_fit_table_plane()` runs vendor `SearchPlane` RANSAC on the latest depth
   frame → `plane=[a,b,c,d]`, persisted to `transform.yaml`.
3. `_depth_at(px,py)` returns **height above the table plane** (not absolute
   depth) when a plane is loaded — so the gate is "object stands above the
   table", filtering hands/reflections. Falls back to absolute Z if no plane.
4. Depth camera_info is latched (TRANSIENT_LOCAL); v5 subscribes with matching
   QoS (Round 17) or `_depth_cam_info` stays None and plane refit fails.

## Pick / place state machine

`sorting_loop()` (detection thread) → `transport_thread()` (motion thread),
coordinated by flags `enable_sorting`, `start_transport`, `transport_info`,
`target`.

- **Lock by LABEL, not instance index (Round 17).** The vendor's LAB blobs
  have stable area-sorted indices; v5's YOLO + `position_reorder` can shuffle
  indices, so matching on `target[1]` (index) broke the lock every frame
  ("blue line but never grabs"). v5 now matches on label and picks the
  candidate nearest the previously locked pixel.
- Stillness gate (vendor-exact defaults, Round 17): a candidate must stay
  within `lock_distance_thresh` (0.005 m, |dx|+|dy|) for `count_still_threshold`
  (10) frames → fire. `detection_avg_frames=1` (instantaneous, like vendor).
- On fire: `transport_info=[pos, yaw, target]`, `start_transport=True`.
  `transport_thread` runs `_do_pick` then `_do_place(label)` then `go_home`.
- Trigger trace in logs: `TRACK ... still=N/10`, then `LOCKED on <label>`,
  then `FIRING TRANSPORT label=...`.

## Force-limited "compliance" grasp

`MotionController.compliance_grasp()` exists, but **the servos cannot report
load or current** at the serial protocol level (only position, voltage,
temperature, torque on/off — confirmed at the bytes in the driver SDK). So
"force limited" is necessarily: stepped close until a **contact-stop** (the
servo stops moving) + a **per-class strength cap** + a **temperature
cutoff**. It returns `gripped`/`closed`/`overheat`/`no_feedback`. The
intended closed-loop behaviour (fail the pick when no contact) may be only
partially wired — see `06_PLANS_DONE_AND_UNFINISHED.md`.

## Domain & launch

Everything runs on **ROS_DOMAIN_ID=0** (pinned by `launch_v5.sh`). The v5
launch includes vendor `depth_camera`, `calibration_node`, and `lab_manager`
so all four nodes share the domain. `launch_v5.sh` stops/disables the
factory `start_app_node.service` first, then waits for the RGB topic (and
soft-waits for depth) before launching.

### The "camera steal" risk (watch this)
Anything that opens the physical Orbbec device a SECOND time can steal it
(one loses the USB claim → camera dead). `lab_manager` and `calibration_node`
only *subscribe* (safe), but `depth_camera` bringup *opens* the device. If
the camera dies after launch, suspect a double-bringup race. `launch_v5.sh`
comments cover reclaiming it via the service.
