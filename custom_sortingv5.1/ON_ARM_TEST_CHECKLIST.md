# v5.1 — On-Arm Test Checklist

Run this on the JetArm (none of it could be verified remotely). Order matters:
**deploy → settings sanity → calibration → detection (OBB) → depth → pick/place
→ presets.** Each item names the fix it verifies and exactly what to watch.

Launch with verbose logs: `JETARM_V5_DEBUG=1 ~/jetarm_v5/launch_v5.sh`
Logs: `~/jetarm_v5/logs/v5_session_*.log` (grep tags in brackets, e.g. `[v4][init]`).

---

## 0. Deploy v5.1 side-by-side with v5
v5.1 installs ALONGSIDE v5 — a separate **"JetArm Sort v5.1"** icon, its own
`~/jetarm_v5_1/` launcher, and distinct ROS entry points (`custom_sortingv51` /
`tune_uiv51`). v5's icon and deployment are left intact as a fallback.
install.sh adds new `setup.py` entries, so it triggers a colcon build (a few
minutes):
```bash
git -C ~/jetarm_v5_src fetch origin && git -C ~/jetarm_v5_src reset --hard origin/main
bash ~/jetarm_v5_src/custom_sortingv5.1/install.sh
~/jetarm_v5_1/launch_v5.sh      # or double-click the new "JetArm Sort v5.1" icon
```
- [ ] A new **"JetArm Sort v5.1"** desktop icon appears, and the old **"JetArm
      Sort v5"** icon is still present (still launches v5).
- [ ] `install.sh` logs `installed …/{slow,medium,fast}.yaml` into
      `~/jetarm_v5_profiles/` and seeds launchers into `~/jetarm_v5_1/`.
- [ ] `ls -l ~/ros2_ws/src/app/app/custom_sortingv51.py` resolves into
      **custom_sortingv5.1/** (and v5's `custom_sortingv5.py` symlink is untouched).
- [ ] Launch v5.1; node, tuner UI, and the vendor `calibration` + `lab_manager`
      nodes all start.
- [ ] `ros2 node list` shows **`/lab_manager`** (NOT `/lab_config_manager`).  ← **A1**

## 1. Settings sanity (do this before blaming anything else)
- [ ] **Engine path** — you mentioned `scaff_cubes_best.engine`, but the shipped
      fallback is `/home/ubuntu/third_party_ros2/data/best_scaff2.engine`. Set
      your real engine on the **Detection tab** (or in `default.yaml`) and Save.
- [ ] **`engine_task`** — if detection produces boxes but **never oriented
      boxes** (see step 4), your engine lacks task metadata: set
      `engine_task: obb` (Detection tab / default.yaml) so ultralytics doesn't
      silently fall back to plain `detect`. This is required for the OBB grasp.
- [ ] **`CHASSIS_TYPE`** — confirmed `Slide_Rails` (the JetArm's lead-screw rail
      axis; the kinematics/pick depend on it — do NOT change it). The boot log
      line `[v4][init] config_path=… (CHASSIS_TYPE='Slide_Rails')` should show
      `…/stepper/config/` as the path.  ← **C1**

## 2. Position calibration  ← A1 / A5 / C1
- [ ] Place an AprilTag (ID 1/2/3/100, tag36h11) flat in view, workspace clear.
- [ ] Position tab → **CALIBRATE POSITION**. Arm moves to its pose, tag is read.
- [ ] UI reports **CALIBRATED (vendor)** within ~35 s (was 20 s — A5).
- [ ] The fresh `transform.yaml` is the one v5 reads: it should be under the path
      printed at boot (`stepper/config` if Slide_Rails, else `app/config`). If
      calibration "succeeds" but the workspace overlay/aim is still wrong,
      suspect **C1** — confirm the write path == the read path.
- [ ] Yellow aim-X sits on a cube placed in the workspace.

## 3. Color calibration  ← A1 (the big one)
- [ ] Color tab → **ENTER**. UI shows **LAB: enter OK** (previously timed out).
- [ ] The mask preview renders from `/lab_manager/image_result` (needs
      `sudo apt install -y python3-pil.imagetk` once).
- [ ] Drag a slider → mask updates live → **STASH** → **SAVE** updates
      `lab_config.yaml`.

## 4. Detection / OBB  ← B4
- [ ] With sorting stopped, the viewer (`/custom_sortingv5/image_result`) draws
      **oriented (rotated) boxes** on cubes and the scaff — not axis-aligned
      rectangles. If they're axis-aligned, fix `engine_task` (step 1).
- [ ] Class names appear in the heartbeat/Classes filter (e.g. `scaff`,
      `red cube`, …).

## 5. Depth  ← A3 / B3
- [ ] Boot log shows `depth source = /depth_cam/depth_to_color (colour-aligned=True)`
      (preferred) or the raw topic if that's all that publishes — but it must
      pick one that **actually delivers frames** (B3). Look for
      `FIRST DEPTH FRAME on <topic>`.
- [ ] Toggle **overlay_depth_view** on: the JET heatmap inside the workspace
      should light up on the cubes/scaff (confirms depth reads the objects).
- [ ] Place an object near the image edge — it should still get a depth value
      and remain pickable (A3; previously edge detections were dropped).
- [ ] Depth tab → **CALIBRATE DEPTH** → "Plane refit OK" + a `plane:` appears in
      `transform.yaml`, or an explicit `PLANE REFIT FAILED: <reason>`.

## 6. Pick & place — one object at a time  ← B1 / A2 / A4
- [ ] Boot log shows `[v4][init] captured endpoint pose t=[...]` (**B1**). If you
      instead see `endpoint pose NOT captured … legacy calibrated path`, the FK
      service isn't responding — picks will use the legacy path (still works,
      less depth-accurate); investigate `kinematics/set_joint_value_target`.
- [ ] Enable sorting on ONE cube. Logs trace `TRACK → LOCKED on <label> →
      FIRING TRANSPORT`. Arm picks it and places it in that class's zone.
- [ ] **B1 check:** the arm descends **onto** the object, not offset to one
      side. (Identity-endpoint bug would land it off-target.) If still offset,
      verify the endpoint was captured and that calibration (step 2) is good.
- [ ] **A4 check:** between grab and drop the arm retreats up/centre before
      traversing — it should not skim other objects at low height.
- [ ] **A2 check (only if `compliance_grasp_enabled: true`):** trigger a grasp on
      nothing / a too-small target → it must **not** proceed to place an empty
      gripper; it re-opens and re-locks. If *every* real grasp also fails with
      `no_feedback`, this arm's gripper servo can't report position → set
      `grasp_fail_on_no_feedback: false`.

## 7. Scaff narrow-edge grasp  ← B4 (the headline goal)
- [ ] Enable sorting on the scaff. The gripper should clamp across its **narrow
      waist** (short axis), not along the length.
- [ ] If it grabs the wrong axis, set **`grasp_yaw_offset_deg: 90`** (try `-90`)
      via a profile/the Detection tab and retry — no code change needed.

## 8. Presets  ← presets (YAML + load_profile) / A6a
- [ ] The Presets bar shows **Slow / Medium / Fast**. Clicking one shows
      `LOADING <name> preset...` then `<NAME> PRESET LOADED`, and the arm
      visibly changes pace (Fast = quicker/looser, Slow = gentle/sequential).
- [ ] These load `~/jetarm_v5_profiles/{slow,medium,fast}.yaml` via the node's
      `load_profile` service (not a live per-key push), so a partial/edited
      preset still applies cleanly. If a button errors, confirm the file was
      seeded (step 0).
- [ ] `fast`/`medium`/`slow` do NOT appear in the custom-preset dropdown (they
      have their own buttons) and can't be overwritten by "Save as preset".

## 9. Robustness spot-checks  ← A6d / P1 / lock / C2
- [ ] **A6d** — hot-swap to an engine with different class names: the Classes
      filter rebuilds and the UI does **not** freeze for a beat.
- [ ] **P1** — with no viewer subscribed, `ros2 topic hz
      /custom_sortingv5/image_result` shows nothing publishing; open a viewer
      and it resumes (~30 Hz). (Overlay compositing is skipped when unwatched.)
- [ ] **Lock / C2** — rapid START/STOP toggling never leaves a stuck
      `start_transport`; and the boot log shows `bringup_depth_camera:=true`
      (it logs `:=false` only if you set `FORCE_SERVICE_RESTART=1`, which you
      should not need — the default path is camera-safe).

---

## Bin Teach & Workspace Map (tuner UI → "Bin Teach" tab)
Fixes placement landing in the wrong spot. STOP sorting first; the arm moves.
- [ ] STOP sorting. Open the **Bin Teach** tab. The class dropdown fills with the
      model's classes once the engine has loaded.
- [ ] **Joint jog** — press J1…J5 ± (step = 10 pulses). The corresponding servo
      moves; the **world (x, y, z)** readout and **servos 1-5** update within ~1 s
      (via `~/teach_status`). Bump the joint step to 20–30 for coarse moves.
- [ ] Reach a bin you could NOT reach with world jog (edge of workspace) using
      joint jog alone — confirms joint jog needs no IK.
- [ ] **World jog** — X/Y/Z ± (step = 0.01 m) nudges in straight lines. If a
      target is unreachable the status says `UNREACHABLE (IK) - not moved` and the
      arm stays put (the jog is reverted).
- [ ] Lower the gripper to just above a physical bin, pick that class in the
      dropdown, **Save here → bin** (confirm). Status shows
      `SAVE <class> = [x, y, z, 'taught']`.
- [ ] **Go to bin** for that class drives the arm back to the saved spot.
- [ ] **Saved bins** list shows a row per class with its coordinate + a **Go**
      button. Press **Go** on a set bin → arm drives there; nudge with jog; **Save
      here → bin** again to fine-tune. The row's coordinate updates after the save.
- [ ] START sorting and drop that class — it lands **at the taught bin** (not the
      old forward-grid spot, and with no affine offset). Repeat per class incl.
      `scaff` → waste bin.
- [ ] If a class is still unmapped, the heartbeat `unmapped` badge flags it.
- [ ] **Workspace map** (optional, after AprilTag CALIBRATE): jog to mat centre →
      **Set centre**; jog to each corner → **Add edge** (×4) → **Save workspace**.
      Status: `world origin re-anchored …`. A centre > 5 cm from the current origin
      is refused until you tick **Confirm** (catches a mis-jog). Verify a couple of
      live picks still land correctly (re-anchoring shifts every detection's X/Y).
- [ ] Confirm `transform.yaml` still has its original `extristric` / `corners` /
      `plane` (only `white_area_pose_world` should change) — detection ROI intact.

## Factory restore + vision/position alignment (this round)
Full procedure in `RESTORE_AND_CALIBRATION.md`. Quick checks:
- [ ] Re-seed config: `cp ~/jetarm_v5_src/custom_sortingv5.1/profiles/default.yaml ~/jetarm_v5_profiles/default.yaml`; boot log shows the factory values and `CHASSIS_TYPE='Slide_Rails'`.
- [ ] `scaff` bin is back to `[-0.076,0.16,0.015]` (NOT the home pose `[0.127,0,0.182]`); all 5 classes on the vendor colour grid.
- [ ] **Clean AprilTag CALIBRATE** (after `restart_v5.sh`): `[roi] loaded plane` normal ≈ `[0,0,-1]` (reject sideways planes).
- [ ] **No grasp into table:** pick log shows `descend … -> z` ≥ 0.01, never negative; no "descend IK failed" on in-reach objects.
- [ ] **No flip:** after grab the arm rises to the carry pose (`[0.11,0,0.15]` on Slide_Rails) and traverses without a back-flip; no `multiple access on port` in the boot log (if seen → USB/serial issue, reseat).
- [ ] **No wrist over-rotation:** a horizontal cube → gripper aligns to the radial approach only (no spurious 90°); the thin scaff → jaws clamp across its narrow axis.
- [ ] **Vision ↔ grab aligned:** the projected aim sits on the object after calibration; any constant residual is removed with `grip_offset_x` only (NOT `workspace_scale`, which stays 1.0).
- [ ] **OBB headroom:** with sorting running, `loop_lag_ms` is low/stable (no climb to 200ms+) and `inference_age_ms` stays small — the loop now processes each inference frame once.
- [ ] Each class sorts to its factory-grid bin; fine-tune exact spots with Bin Teach if your physical bins differ.

## Pick alignment (grab lands off-target)
On the **Bin Teach** tab, "Pick alignment" panel. These are PICK (grab) offsets,
separate from the bins.
- [ ] Run sorting and watch a grab. If it lands, e.g., +7 cm to the right and
      too low (pushes into the table):
- [ ] Nudge **X –** until the grab centres left-right (a +7 cm-right error ≈
      `grip_offset_x = -0.07`), and **Z +** until it stops driving into the table
      (≈ `grip_offset_z = +0.03`). Each nudge applies to the **next** pick.
- [ ] Re-run a grab after each nudge; when it grabs dead-on, press **Save
      alignment** (persists `grip_offset_x/y/z` to default.yaml).
- [ ] Confirm depth is NOT the Z cause here: the pick logs `worldpos=legacy/table`
      and the grab height is a fixed constant, so the Z fix is `grip_offset_z`,
      not the depth camera.

## If a pick fires but the arm doesn't move
Look downstream (unchanged logic, but now fed correct world coords): `_do_pick`
IK reachability, `_apply_kinematics_calibration` offsets from `calibration.yaml`,
and (if enabled) `compliance_grasp`. Compare against vendor
`object_sorting.py` / `pick_and_place.py`.

## Rollback
v5.1 is additive; the original `custom_sortingv5/` is untouched. To revert,
install/relaunch from `custom_sortingv5/` instead.
