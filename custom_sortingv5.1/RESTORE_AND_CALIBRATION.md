# v5.1 — Factory Restore + Calibration Runbook

One app does everything: YOLO‑OBB sorting, AprilTag (position) calibration, LAB
colour calibration, and Bin Teach. This is the procedure to get back to a
known‑good, factory‑aligned state and (re)calibrate.

## What the code fixes changed (vs the drifted build)
- **World map = AprilTag only.** Removed the `workspace_scale` multiply from the
  pick world math (`_apply_world_offsets`). Position now comes purely from the
  AprilTag extrinsic + the `calibration.yaml` `pixel`/`kinematics` affines, with
  `grip_offset_x/y` as the *only* manual nudge (additive metres; `grip_offset_z`
  applied after the kinematics affine). Keep `workspace_scale = 1.0`.
- **No grasp into the table.** `min_descend_z_m` default → **0.01** (the vendor
  grab height), so `descend_z = max(obj_top_z − grab_depth, 0.01)` can never go
  negative. Keep `grab_depth = 0.02` (per‑class deeper via `target_overrides`).
- **No flip on traverse.** The carry/transit pose now branches on chassis like
  the vendor: **Slide_Rails → `[0.11, 0, 0.15]`**, else `[0.11, 0, 0.09]`, pitch 73.
- **No wrist over‑rotation on cubes.** `grasp_short_axis_min_ratio` → **0.30**, so
  only the elongated scaff uses the short‑axis clamp; near‑square cubes use the
  vendor min‑|yaw| choice.
- **OBB headroom.** The sorting loop now processes each inference result **once**
  (was re‑running detection on the same frame ~1000×/s, starving inference). With
  `detection_avg_frames = 1`, locks accumulate per inference frame (vendor cadence).

> **Calibration never overwrites these settings.** AprilTag CALIBRATE writes only
> `transform.yaml` (the world frame); the tunables live in `default.yaml`. The
> earlier drift came from UI "Save" actions, not calibration.

## 1. Restore factory defaults (on the Jetson)
```bash
git -C ~/jetarm_v5_src fetch origin && git -C ~/jetarm_v5_src reset --hard origin/main
bash ~/jetarm_v5_src/custom_sortingv5.1/install.sh
# Re-seed the boot config to the clean factory baseline:
cp ~/jetarm_v5_src/custom_sortingv5.1/profiles/default.yaml ~/jetarm_v5_profiles/default.yaml
```
Confirm in `~/jetarm_v5_profiles/default.yaml`: `workspace_scale: 1.0`,
`grip_offset_x/y/z: 0.0`, `grab_depth: 0.02`, `min_descend_z_m: 0.01`,
`motion_speed: 1.5`, `aggression: 1.3`, `detection_avg_frames: 1`, and
`place_positions` = the vendor colour grid (scaff `[-0.076,0.16,0.015]`, red
`[0.064,0.23]`, green `[-0.006,0.23]`, light blue `[-0.076,0.23]`, dark blue
`[-0.006,0.16]`). **Model settings are left untouched** (`engine_path`,
`engine_task`, `yolo_*`, `inference_max_hz`).

## 2. Launch (one app) and confirm the environment
```bash
~/jetarm_v5_1/launch_v5.sh          # or the "JetArm Sort v5.1" desktop icon
```
Boot log MUST show `config_path=…/stepper/config/ (CHASSIS_TYPE='Slide_Rails')`.
If it says `None`, the Hiwonder env wasn't sourced — fix that first (wrong env =
wrong calibration files = misaligned picks).

## 3. Calibration — in‑app (1‑to‑1, normal use)
The vendor `calibration` + `lab_manager` nodes run inside the v5.1 stack; the UI
drives them over `/calibration/*` and `/lab_manager/*`.

**Position (AprilTag):**
1. The vendor calibration node throws `InvalidHandle` on re‑entry, so **start
   fresh before each pass**: `bash ~/jetarm_v5_src/custom_sortingv5.1/restart_v5.sh`.
2. Place ONE AprilTag (id 1/2/3/100, 2.5 cm, tag36h11) flat on the mat, fully in
   view, dead steady; clear the arm's path.
3. UI **Position tab → CALIBRATE**. It writes `transform.yaml` and rebuilds the ROI.
4. **Trust it only if** the boot/roi log `[roi] loaded plane` normal ≈ `[0,0,-1]`
   (not sideways). If CALIBRATE says "vendor calibration node not running", the
   node crashed → `restart_v5.sh` and retry.

**Colour (LAB):** UI **Color tab** → ENTER → drag L/A/B min–max while watching the
live mask → **Stash** → **Save to disk** (writes `lab_config.yaml`). (Only needed
if you run the colour path; YOLO‑OBB sorting does not use LAB.)

## 4. Calibration — standalone route (fallback)
If you prefer to calibrate outside the sorting app, or the in‑app node misbehaves:
```bash
# Stop the sorting stack (Ctrl+C its window) AND make sure the factory service
# isn't also holding the serial/camera:
sudo systemctl stop start_app_node.service
# Option A: just relaunch the v5.1 stack fresh and use the CALIBRATE button.
bash ~/jetarm_v5_src/custom_sortingv5.1/restart_v5.sh
# Option B: run ONLY the vendor calibration node from the workspace, calibrate,
#           then relaunch the v5.1 stack.
```
**Rule:** never run the factory `start_app_node.service` AND the v5.1 launch at the
same time — both open the servo serial + the Orbbec, causing
`multiple access on port` (the controller drops the arm mid‑move = the lurch you saw).

## 5. Seeing depth + colour + RGB separately
All streams already publish; view them without touching the app:
- RGB + overlay: `/custom_sortingv5/image_result` (or browser `http://<jetson>:8080`).
- Raw colour: `/depth_cam/rgb/image_raw`.
- Raw depth: `/depth_cam/depth/image_raw`.
```bash
ros2 run rqt_image_view rqt_image_view        # pick any of the topics
```

## 6. Bins
The factory default is the vendor colour grid (above). To match YOUR physical
bins exactly, use **Bin Teach** (jog → Save) per class — it overrides
`place_positions` and is independent of calibration.

## 7. Hardware note (the lurch)
The on‑arm fling coincided with a USB/serial dropout, not software. Run the arm's
controller USB on its own port (away from the camera hub) and check power before
trusting motion near your hands; keep the e‑stop ready.
