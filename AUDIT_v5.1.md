# JetArm v5.1 — Full Application Audit

**Scope:** `custom_sortingv5.1/` (node 4,831 L + UI 3,410 L), launch file, all
deployment scripts, all YAML/config paths, and the vendor interfaces they drive.
**Method:** six parallel read-only auditors (yaml-persistence, ui↔node wiring,
sorting pipeline, calibration, deployment, dead-code/quality), each verifying
claims against real code with line numbers.
**Status:** review only — **no code was changed to produce this document.**
**Audited at commit:** `87c202c1`

---

## How to read this

Every finding has: **what** (the defect), **trigger** (the exact action that hits
it), **impact** (what you see/lose), **fix** (smallest correct change — *not
applied*).

Severity means:
- **CRITICAL** — data loss, an arm-safety hazard, or a feature that is 100% dead.
- **HIGH** — silently wrong behaviour, or a setting that doesn't do what it says.
- **MEDIUM** — real but narrower, or needs a specific sequence.
- **LOW** — cosmetic, confusing, or hygiene.

---

## §0 — STOP-SHIP: fix these before the next run

### 0.1 CRITICAL — `_place_by_joints` is 100% dead; it drops the cube at home
`custom_sortingv5.py:3723, 3750, 3758` — **introduced by me in Round 20b.**

`MotionController.aborted` is a **`@property`** (`:505-507`), but I call it as a
function in three places. `self.motion.aborted()` evaluates `False()` →
`TypeError: 'bool' object is not callable`. Proven at runtime.

The TypeError escapes `_do_place` into `transport_thread`'s except handler →
`go_home(True)` → **which opens the gripper** → the cube is dropped at home and
logged only as "transport cycle CRASHED".

**This exactly reproduces the symptom this whole fix round was meant to cure**,
on every taught bin where hover/descend IK is refused — i.e. precisely the case
the joint fallback exists for.

- **Fix:** delete the parentheses at all three sites (existing correct usages at
  `:3524` and `:3638` have none). Three characters.

### 0.2 CRITICAL — `_transport_busy` leaks `True`, freezing Bin Teach forever
`custom_sortingv5.py:3884 / 3914 / 3959` — **also introduced by me in Round 20b.**

Set `True` at the top of the transport cycle, cleared at the bottom — but the
**CONTROLLER DOWN branch `continue`s** at `:3914` and skips the clear.

- **Trigger:** any USB/servo-controller dropout during sorting (you have had these).
- **Impact:** every teach command is refused with "a pick/place cycle is still
  unwinding" until the node is restarted.
- **Fix:** wrap the cycle body in `try/finally`.

### 0.3 CRITICAL — Depth "plane refit" can destroy your entire calibration
`custom_sortingv5.py:2081-2088`

The read is wrapped in `except Exception: data = {}`, then it `safe_dump`s the
whole file. Any transient read error (mid-write by the vendor node, truncation,
permissions) leaves `transform.yaml` containing **only** `{plane: [...]}`.

- **Impact:** `extristric`, `corners`, `white_area_pose_world`,
  `white_area_pose_cam` destroyed → `get_roi` fails forever → picking dead →
  workspace teach refuses → full AprilTag re-calibration required.
- **Compounding (`:2068-2088`):** there is **no mutual exclusion** between the four
  writers of `transform.yaml` (vendor calibration, plane refit, workspace teach,
  ROI reload). Plane-refit has no `_calibrating` guard, so pressing it during an
  AprilTag calibrate is a lost-update race.
- **Fix:** abort on read failure (only write when the load yielded a dict
  containing `extristric`); write via temp file + `os.replace`; refuse refit while
  `_calibrating`.

### 0.4 CRITICAL — With no depth plane fit, **every detection is silently dropped**
`custom_sortingv5.py:3190-3196` + `:4394-4417` + `:874-875`

`_depth_at` returns *height above plane* only when **both** `table_plane` and
`_depth_cam_info` exist. Otherwise it returns **absolute depth** (~0.6–1.0 m),
which always fails the sanity gate `-0.02 ≤ z ≤ 0.20`.

- **Trigger:** `transform.yaml` has no `plane` key (or depth camera_info hasn't
  arrived) while `use_depth_for_z` is on — **which is the default**.
- **Impact:** `target_info` empties → no overlay boxes, no locks, **no picks at
  all** — and the heartbeat still reports healthy. This is a prime suspect for
  "sorting just stops working".
- **Fix:** skip the gate unless plane + caminfo are both present.

### 0.5 CRITICAL — `restart_v5.sh` kills itself and never relaunches
`restart_v5.sh:21`

`pkill -f "custom_sortingv5"` matches its own parent bash (whose argv is the
documented `bash ~/jetarm_v5_src/custom_sortingv5.1/restart_v5.sh`).

- **Impact:** the stack is torn down, `exec launch_v5.sh` at `:43` never runs, and
  the arm is left dead with the factory service stopped.
- **Fix:** anchor the pattern (`app/lib/app/custom_sortingv5`) or exclude `$$`/`$PPID`.

### 0.6 CRITICAL — `update_and_reseed.sh` destroys 47 of 54 tuned settings
`update_and_reseed.sh:17, 38` — see the full table in §6.

Preserves only 7 model keys; overwrites `default.yaml` wholesale. **All taught
bins, all offsets, all grasp/motion tuning → factory.** Only the timestamped
backup saves you.

### 0.7 CRITICAL — `install.sh` deploys `origin/main`, not the tree you ran it from
`install.sh:113, 116, 123`

It hard-resets `~/jetarm_v5_src` to `$BRANCH` (default `main`) and links from
there — ignoring the checkout the script itself lives in. `update_and_reseed.sh:34`
even rewrites the file bash is mid-read of.

- **Impact:** on-device edits silently discarded; a feature branch never deploys.
- **Fix:** default `SRC_DIR` to the script's own repo root, or refuse a dirty tree.

### 0.8 CRITICAL — The UI can never detect a rejected setting
`tune_uiv5.py:409-415`

`set_value()` does `res = self._wait_future(...)` — which returns a **bool** — then
calls `res.results[0].successful`, which always raises; the `except` returns `None`.

- **Impact:** **every** parameter write reports success even when the node rejects
  it (out of range, wrong type). Explains any "I changed it and nothing happened".
- **Fix:** `res = future.result()` after the wait.

---

## §1 — Persistence: what saves, what silently doesn't

### Verified correct
- `_merge_into_default` re-reads from disk before merging — two tabs' saves can't
  erase each other (`:1553-1573`).
- 5-field taught entries survive the YAML round-trip **byte-exact**, and other
  keys are preserved (proven by execution, 3,000 fuzz cases).
- Boot order is right: `_apply_default_profile_seed` runs before declare; the
  seeded value survives (`:1394-1400`).
- Speed presets only set the 13 keys they contain — they don't touch bins/engine.
- `load_profile` applies params one at a time, so one bad key can't abort the rest.

### HIGH — Places tab "Save" persists nothing
`tune_uiv5.py:2869, 2911, 2844` — row Save, Save-all and Test-grip all use
`set_value` (runtime only) while the status says "TARGET … SAVED".
**Impact:** coordinates and per-class `grasp_strength` revert on restart. Only the
bottom "Save & Apply" bar persists. **Fix:** relabel to "Apply", or route through
`apply_and_persist(persist=True)`.

### HIGH — Class-filter checkboxes can silently wipe your saved filter
`tune_uiv5.py:2228-2251, 2156` — the grid builds before the persisted list arrives,
so every class ticks on; a later Detection "Save & Apply" writes `'[]'`.
**Impact:** disabled classes start being picked again. **Fix:** don't include
`yolo_enabled_classes` in the payload until the cache is warm.

### HIGH — `apply_and_persist` reports success when every key was rejected
`custom_sortingv5.py:2946-2957` — rejected keys only hit `_stage`; `response.success`
is unconditionally `True`. **Fix:** `success = (len(applied) == len(cfg))`.

### MEDIUM — Same stale-cache class of bug still live on two other params
`tune_uiv5.py:1478, 2723, 2386, 2594` — `_grasp_strength` and `_pick_off` are
startup snapshots never refreshed (this is exactly the `place_positions` bug fixed
in Round 20b, unfixed on these two). A 2 s `get_values` timeout at startup →
`_grasp_strength = {}` → Places Save & Apply writes `'{}'`. Nudging offsets on Bin
Teach then silently reverts the Position tab's values. **Fix:** mirror `_live_places`.

### MEDIUM — YAML scalar type freezes the slider
`custom_sortingv5.py:1394-1428` — pre-declare `set_parameters` succeeds (undeclared
params allowed), `declare_parameter` then raises AlreadyDeclared and is swallowed,
so the range descriptor is **never installed** and the type is whatever YAML gave.
Hand-edit `grab_depth: 0` (int) → that slider is permanently inert.

### MEDIUM — Speed presets are silently undone by the next tab save
`tune_uiv5.py:3172-3190` — preset loads into the node but never refreshes the
widgets; `_on_tab_save` reads widgets → old values re-persisted.

### MEDIUM — `save_as_default` replaces rather than merges
`custom_sortingv5.py:3131, 1544` — rewrites from `TUNABLE_PARAMS` only, dropping
comments, formatting and any non-tunable key, and **bakes the current
`calibrate_overlay_mode` in as the boot value** (press it with the overlay up and
you boot with a permanent overlay).

### MEDIUM — Non-atomic writes; a corrupt `default.yaml` boots silently to factory
`custom_sortingv5.py:745-749, 737` — `open(w)` truncates before dump; a parse error
returns `{}` with **no log**. A power cut mid-save = every setting silently gone.

### MEDIUM — `enable_camera_sub` / `enable_inference` persist but are ignored at boot
`custom_sortingv5.py:1259, 1516` — seed runs before `self.inference` exists.

### MEDIUM — LAB "Save" always claims success and no-ops without a prior Stash
vendor `lab_manager.py:163, 184` — `save_to_disk` uses `self.param_name`, set only
by `stash_range`; the AttributeError is caught and success returned anyway.

---

## §2 — UI ↔ node wiring: placebos, orphans, mislabels

### PLACEBO CONTROLS — the UI writes it, nothing reads it
| Control | Where | Reality |
|---|---|---|
| **`workspace_scale`** ("Workspace scale (Z)") | `tune_uiv5.py:190` | Node comment: "no longer affects picks". Only reference is that comment. Still persisted — bakes a lie into `default.yaml`. |
| **`inference_warmup`** checkbox | `tune_uiv5.py:213` | Node warms up unconditionally (`:388-395`), never reads the param. |
| **Per-class `grasp_strength` + "Test grip"** | Places tab | Only consumed inside `if use_compliance:` (`:3577`); `compliance_grasp_enabled` **defaults false**. Test Grip *always* uses compliance — so it confirms a value real picks ignore. |
| **`workspace_size_x/y`** | Calibrate tab | Sizes the overlay rectangle only (`:4552`); the real ROI is built independently. |
| **launcher "camera-off"/"ai-off" modes** | `launch_v5.sh:211-213` | Args are passed but the launch file never declares or forwards them. Camera/AI keep running. |

### HIGH — Mislabelled: `detection_offset_x/y` moves the arm, not just boxes
`tune_uiv5.py:140` says "Overlay offset — nudge boxes"; the node (`:3177-3182`)
states the same coords feed the world projection. You nudge cosmetically and the
arm moves.

### HIGH — `grip_offset_x/y` has two competing controls
Calibrate-tab sliders vs Bin-Teach "Pick alignment" ± buttons, independent state,
neither refreshes the other. Using one silently undoes the other.

### HIGH — Position tab shows `workspace size : ? x ? m` forever
`tune_uiv5.py:630` reads heartbeat keys the node never publishes.

### ORPHANED NODE FEATURES (fully built, no UI path)
- **`target_overrides`** (`:939`, `_per_target_overrides` `:3485`) — per-class
  gripper pulse / duration / settle / grab-depth / grasp-step overrides. Complete
  implementation, **zero UI**, absent from `default.yaml`.
- Services `~/set_target` (`:1090`), `~/save_yolo_config` (`:2960`) — no caller.
- Params with no UI at all: `min_descend_z_m`, `safe_transit_enabled`,
  `grasp_fail_on_no_feedback`, `grasp_yaw_offset_deg`, `calibrate_flash_secs`,
  `test_grip_dwell`, `debug`.
- `startup_self_calibrate` — declared, no UI, node documents it as a no-op.

---

## §3 — Sorting pipeline

### Verified correct
- Handoff under `_transport_lock` with a copied position (no aliasing) `:3874-3888`.
- Kinematics affine applied **exactly once**, correctly skipped for taught bins.
- Controller-alive guard both before and after the pick, demoting fake success.
- Label-only lock with nearest-pixel disambiguation — the Round-17 fix is genuine.

### HIGH — `compliance_grasp` timeout can't cover full jaw travel, and reports "empty"
`:677, 720, 3618` — 200→540 at step 15 ≈ 23 steps × (0.10 dwell + up to 0.6 s
readback) vs a 2.0 s `grasp_timeout`. Deadline-exit returns `'closed'`, which
`_do_pick` treats as *no object* → **opens the gripper and bails, releasing an
object it may actually be holding**.

### HIGH — Transport-thread death is never detected
`:3836`, heartbeat `:1661` — only `_sorting_thread` liveness is reported. If the
transport thread dies, `start_transport` stays `True` → the sorting loop's gate
blocks detection **forever**; overlays freeze, picks stop, one CRASHED line.

### HIGH — START clears the abort with no in-flight guard
`:1998` — STOP mid-pick then START resumes the frozen cycle from an arbitrary pose.
(Round 20b guarded this for *teach* but not for START.)

### HIGH — `detection_history` is never cleared per pick
`:4127` — keyed by label, trimmed to `detection_avg_frames`, not to the instance.
With two same-class objects the fired `avg_pos` averages a just-removed instance
with the new one → **the arm descends between two cubes and grabs nothing**.

### MEDIUM — Unguarded `config_data['kinematics']`
`:3481, 3783` with `_calibration_cfg` returning `{}` on OSError → KeyError. In
`_do_place` this fires **with the object held** → dropped at home.

### MEDIUM — `position_reorder` discards all detections when nothing matched
vendor `position_change_detect.py:37` — unmatched points are only re-added
`if new_points != []`. Fast-moving/jittery objects → empty frames → locks never mature.

### MEDIUM — `target_miss_count` never reset on a good track
`:4157` — sparse misses accumulate and eventually drop a healthy lock.

### LOW — Overlay aim marker omits corrections the pick applies
`:4656` vs `:3888` — excludes the kinematics affine and `grip_offset_z`, so the
marker can sit dead-on while the arm reaches ~5%+offset away. Also `_pixel_to_world`
(`:3144`) is **dead code**; the legacy path hardcodes `position[2] = height`, so
without the vendor depth path **grab height never varies**.

### LOW — `e_distance` is Manhattan, not Euclidean
`:4103` — `sqrt(dx²)+sqrt(dy²)`; `lock_distance_thresh` is ~1.4× tighter diagonally.

---

## §4 — Calibration (four subsystems)

### HIGH — The Colour (LAB) tab has **no effect on v5 sorting**
`custom_sortingv5.py:3197` — v5.1 detection is YOLO-only. **No file under
`custom_sortingv5.1/` reads `lab_config.yaml`.** The tab tunes the vendor node for
*other* Hiwonder apps. You can spend a session "calibrating colour" for nothing.

### HIGH — LAB `ENTER` leaks a camera subscription every press
vendor `lab_manager.py:106` creates a new `/depth_cam/rgb/image_raw` subscription
per call without destroying the old one, and the UI never calls `lab_exit` on
window close. **N subscribers + mask publishing forever — camera contention with YOLO.**

### HIGH — The CALIBRATE status line lies
`tune_uiv5.py:3106` — waits ~12 s against a 35 s worker and accepts any
`last_calibrate` without checking it is newer than the press. **First run always
shows "TIMED OUT" while still running; the next run shows the previous result.**

### HIGH — Calibrate timeout leaves disk and memory inconsistent
`:2148` — vendor writes the YAML *then* publishes finish. On timeout v5 doesn't set
`start_get_roi`, so new geometry is on disk while the old extrinsic/ROI/plane stay
in memory. Picks use a stale world map until restart.

### HIGH — Workspace teach moves the origin but not corners / ROI / plane
`:2893` — rewrites only `white_area_pose_world`, **with identity rotation** (discards
tilt). `corners` (what `get_roi` actually projects) and `plane` are untouched →
overlay and workspace math disagree.

### HIGH — Teach and AprilTag silently clobber each other
vendor `calibration.py:205` unconditionally rewrites `white_area_pose_world`. A
later CALIBRATE discards a taught origin with **no warning**, and vice versa.

### HIGH — Taught origin affects PICK only on the legacy path
`:3334-3394` — the vendor depth path computes world from `table_plane` + `hand2cam`
+ K and never reads `white_area_center`. On a colour-aligned depth rig the taught
origin changes the **overlay but not the grab**.

### MEDIUM — Depth heatmap toggle does nothing by default
`:4488` — the heatmap lives inside `_draw_calibration_overlay`, which returns when
`calibrate_overlay_mode == 'off'` (the default). The Depth tab never sets the mode
but tells you to enable the heatmap.

### MEDIUM — The heatmap is not plane-relative and is stretched
`:4602` — comment claims plane-relative but uses absolute depth; raw 640×400 is
resized to 640×480. **The tool used to confirm depth/RGB alignment fakes alignment.**

### MEDIUM — Plane fitted with the wrong intrinsics when the aligned stream wins
`:2216, 4322` — always uses `/depth_cam/depth/camera_info` even when the frame came
from `depth_to_color` (colour intrinsics/size) → wrong above-plane heights.

### MEDIUM — Plane never invalidated by an AprilTag recalibration; and RANSAC output is unchecked
`:1873`, `:2198` — no normal-direction/inlier sanity check, no guard against objects
or the arm being in frame. A plane fitted to a cube face silently corrupts all
height gating.

### MEDIUM — The ROI is computed but never gates detections
`:3161-3242` — `roi` is passed in and ignored in the body; it is only a boolean
"ready" flag. **Objects outside your taught workspace are still picked.**

### LOW — The 5 cm workspace guardrail is XY-only and *relative*
`:2881` — Z unbounded, and it compares against the *current* origin, so repeated
4.9 cm saves walk the origin indefinitely.

---

## §5 — Dead code, silent failures, thread hazards

### Dead code (node)
`place_offsets` (`:1041`) · `YOLO_CONFIG_PATH` (`:228`) · `self.lock` — an RLock
created and **never acquired** (`:996`) · `self.fps` (`:997`) · `self.tag_size`
(`:1024`) · `MotionController.node` (`:497`) · `camera_info_sub` handle (`:1262`) ·
**`_pixel_to_world` — zero callers** (`:3144`) · unused imports (`functools`, six
`rcl_interfaces` names, `ServoPosition`).

### Dead code (UI)
`call_recalibrate` · `call_exit` (**the UI never sends `~/exit`**) · `trigger_service`
· `lab_get_all_names` · `call_save_yolo_config` · `list_cli` · `_img_previews`
(comment says it keeps refs alive; nothing is ever appended).

### HIGH — Tk widgets mutated from worker threads
`tune_uiv5.py:2051, 2966, 3015, 3225` — daemon threads call `StringVar.set`,
`_set_status`, Listbox delete/insert, and `messagebox` directly. **Tcl is not
thread-safe** — a real source of intermittent tuner hangs/crashes. The correct
pattern (`root.after(0, ...)`) is already used at `:2012`.

### HIGH — Blocking ROS calls on the Tk main loop
`tune_uiv5.py:1239, 1300, 976, 1094, …` — `set_value`/`get_values` block up to 2 s
from slider release / Return / checkbox handlers. **The GUI freezes ~2 s per slider
release**; tab construction can stall ~10×.

### HIGH — `_init_state()` re-runs on every `~/enter` while worker threads are live
`:1577, 1956` — resets `target`, `transport_info`, `start_transport`,
`detection_history`, and the `_test_grip_running` / `_teach_running` reentrancy
guards, all **outside** `_transport_lock`. Pressing START mid-cycle can start a
second test-grip/teach thread on top of a running one.

### HIGH — `exit_srv_callback` mutates the handoff without the lock
`:1979` — sets `start_transport = False` unlocked and never clears
`transport_info`/`target`, so stale info can be re-consumed on the next enable.

### MEDIUM — `place_positions` is parsed three divergent ways
`:3670` (`len≥3`), `:3691` (`len≥4`+tag), `:3704` (`len≥5`) — each re-parses with
different validity rules. A 4-element taught entry with a malformed 5th field:
`_is_taught_place` says taught, `_taught_joints` returns None → the place takes the
IK path the taught flag exists to avoid. **Fix:** one parse/validate helper.

### MEDIUM — Image-preview `after()` loop and subscription never torn down
`tune_uiv5.py:1564` — reschedules forever; the ROS subscription survives window
close; the inner `except: pass` hides the TclError. CPU/memory grow per window.

### Dangerous silent-failure points
- `_declare_tunables` swallows **every** declare error (`:1424`) — a typo'd default
  silently drops the param; the first `self.p(name)` then raises inside a motion thread.
- Depth-Z failure silently reverts to the hardcoded table height, no log (`:3149`).
- Malformed `place_positions` silently downgrades a taught bin to IK placement (`:3701`).
- `_set_status` swallows TclError — after window close, worker threads fail invisibly.

---

## §6 — Deployment

### WHAT `update_and_reseed.sh` DESTROYS (54 keys total)

**PRESERVED (7):** `engine_path`, `engine_task`, `yolo_conf_thresh`,
`yolo_iou_thresh`, `yolo_max_det`, `inference_max_hz`, `yolo_enabled_classes`.

**DESTROYED (47):**
- **Place/teach:** `place_positions` (**all taught bins**), `target_overrides`
- **Calibration nudges:** `detection_offset_x/y`, `grip_offset_x/y/z`,
  `workspace_scale`, `workspace_size_x/y`, `calibrate_overlay_mode`, `calibrate_flash_secs`
- **Lock gating:** `lock_distance_thresh`, `count_still_threshold`,
  `count_move_threshold`, `detection_avg_frames`
- **Depth:** `use_depth_for_z`, `depth_window_px`, `depth_min_z_m`, `depth_max_z_m`,
  `overlay_depth_view`
- **Grasp orientation/safety:** `grasp_prefer_short_axis`, `grasp_short_axis_min_ratio`,
  `grasp_yaw_offset_deg`, `grasp_fail_on_no_feedback`, `safe_transit_enabled`
- **Motion:** `motion_speed`, `aggression`, `hover_height`, `approach_dwell`,
  `parallel_base_motion`
- **Gripper:** `gripper_open_pulse`, `gripper_close_pulse`, `gripper_close_duration`,
  `gripper_settle`, `grab_depth`, `min_descend_z_m`
- **Compliance beta:** `compliance_grasp_enabled`, `grasp_strength`, `grasp_step_pulse`,
  `grasp_step_dwell`, `grasp_stall_pulse`, `grasp_timeout`, `grasp_max_temp`, `test_grip_dwell`
- **Inference misc:** `inference_warmup`, `hot_log_inference_ms`

Also overwrites `slow/medium/fast.yaml`. Does **not** touch `transform.yaml`,
`calibration.yaml`, `lab_config.yaml`. Backups accumulate at `$DEF.bak.$ts` and
`/tmp/default.yaml.preupdate.$ts` forever.

**Fix:** invert the logic — preserve everything in the backup, merge in only *new*
factory keys. (Or at minimum add `place_positions grasp_strength target_overrides
grip_offset_* workspace_size_*` to `MODEL_KEYS`.)

### HIGH — Other deployment defects
- **Factory-service stop failure is non-fatal** (`launch_v5.sh:232`) — bringup keeps
  `/dev/ttyUSB0` + the Orbbec while our `sdk_launch` opens them → "multiple access
  on port", arm drops mid-motion. Should be fatal.
- **`FORCE_SERVICE_RESTART` interlock covers only the camera** (`:444`) — `sdk_launch`
  and `web_video_server` (:8080) still double-start.
- **`update_and_reseed.sh` has no `set -e`** — an `install.sh` failure is ignored and
  it reseeds `default.yaml` anyway.
- **Build failure leaves an unlaunchable workspace** (`install.sh:642`) — `rm -rf
  build/app install/app` runs *before* colcon; on failure the previously working
  install is already destroyed.
- **v5 and v5.1 are not truly separable** — both name the node `custom_sortingv5` and
  share `~/jetarm_v5_profiles`, `~/jetarm_v5/logs`, `~/.jetarm_v5.env`. `install.sh`
  cleans v2/v4/v4.1 only, so v5's desktop icon survives and **you can launch v5 while
  editing v5.1**; the two write the same `default.yaml` with different key sets.
- **`tools/calibration_tools.sh` can't run under v5.1** — it calls `ros2 run app
  tune_uiv5`, but v5.1 registers `tune_uiv51`.

### MEDIUM
- `update_and_reseed.sh` sed preservation is not value-safe (`&`, `|`, backslashes
  corrupt `engine_path`; the "preserved" echo prints even when nothing substituted).
- `install.sh` keeps a stale v5 `default.yaml`, so upgrading never seeds new v5.1 keys.
- Vendor calibration include hard-fails outside `launch_v5.sh` (`os.environ['need_compile']`
  with no default → KeyError kills the whole launch).
- Hidden `sudo` password prompts appear as a hang (`2>/dev/null`).

---

## §7 — Required order of operations (until the above are fixed)

1. **Stop sorting.** Confirm the camera is streaming (`roi=ok` on the heartbeat).
2. **POSITION (AprilTag) CALIBRATE first** — everything depends on
   `extristric`/`corners`. **Watch the node log, not the UI status line** (it lies
   for ~35 s).
3. **DEPTH plane refit second**, on a clear table with the arm out of frame. **Never**
   press it while a Position calibrate is running. Back up `transform.yaml` first.
4. **WORKSPACE teach last.** Re-running step 2 afterwards silently discards it.
5. **Re-teach place bins after any origin change** — taught bins are absolute and are
   not moved by calibration.
6. **COLOUR (LAB) is optional and has no effect on v5 sorting.** If you open it,
   always press EXIT before closing the window (subscription leak).
7. Verify with `calibrate_overlay_mode = manual` — the only mode that renders the
   workspace box and the heatmap.
8. **Use `install.sh`, never `update_and_reseed.sh`** unless you intend to lose §6.

---

## §8 — Suggested fix order

**Tier 1 (before the next run):** 0.1 (3 chars) · 0.2 (`try/finally`) · 0.3
(abort-on-read-failure + atomic write) · 0.4 (gate skip when no plane) · 0.5
(anchor pkill) · 0.6 (invert MODEL_KEYS).

**Tier 2 (silently wrong):** 0.8 `set_value` · Places "Save" persists nothing ·
class-filter wipe · `apply_and_persist` false success · compliance timeout releasing
a held object · transport-thread death undetected · `detection_history` not cleared.

**Tier 3 (honesty/UX):** label or remove the placebo controls · fix the CALIBRATE
status line · mark the Colour tab as vendor-only · fix the workspace-size readout ·
expose `target_overrides` or delete it.

**Tier 4 (hygiene):** thread-safe Tk marshalling · non-blocking ROS calls on the main
loop · atomic YAML writes · single `place_positions` parser · delete dead code.

---

*No fixes in this document have been applied. Every line number refers to commit
`87c202c1`.*
