# 02 — Reference Index (everything you can use)

## GitHub

- **Repo:** `calo-lategan/new-repo2`
- **Active branches:**
  - `main` — authoritative.
  - `claude/optimize-jetarm-performance-2HqF5` — the development feature
    branch (every round is pushed to BOTH this and `main`).
  - `jetarm-logs` — **dedicated branch for device session logs** (PUSH LOGS /
    `tools/push_logs.sh` push here, never `main`).
  - Many `claude/v4*` / `claude/v4.1*` branches exist — historical, ignore.
- **Dev-branch rule:** develop on `claude/optimize-jetarm-performance-2HqF5`,
  push to it AND `main`. Commit trailers used this session:
  `Co-Authored-By: Claude ...` + `Claude-Session: ...`. Do NOT put any model
  identifier in commits/PRs/code.

## JetArm device directories (paths this project uses)

| Path | What |
|---|---|
| `~/jetarm_v5_src` | The cloned repo on the device. Update target. |
| `~/ros2_ws/src/app/` | Vendor `app` ROS package. v5 sources are **symlinked** in: `app/app/custom_sortingv5.py`, `app/app/tune_uiv5.py`, `app/launch/custom_sorting_nodev5.launch.py`. |
| `~/ros2_ws/src/app/config/transform.yaml` | Position+depth calibration output (extrinsic, corners, plane). |
| `~/ros2_ws/src/app/config/lab_config.yaml` | LAB color thresholds (vendor lab_manager writes this). |
| `~/ros2_ws/src/app/config/calibration.yaml` | Kinematics/pixel offset+scale corrections. |
| `~/jetarm_v5_profiles/default.yaml` | The boot param source of truth (tuner Save & Apply merges here). |
| `~/jetarm_v5/` | Installed launchers: `launch_v5.sh`, `image_view_chain.sh`, `re-enable-factory.sh`. |
| `~/jetarm_v5/logs/` | Session logs: `v5_session_<pid>_<ts>.log`, `v5_ui_<pid>_<ts>.log`. |
| `~/third_party_ros2/data/*.engine` | TensorRT YOLO engines (e.g. `scaff_cube_best.engine`). |

Note: the device user is `ubuntu` (paths shown as `/home/ubuntu/...` in code
constants, `~` in docs). `config_path` constant in the node =
`/home/ubuntu/ros2_ws/src/app/config/`.

## ROS topics

| Topic | Type | Notes |
|---|---|---|
| `/depth_cam/rgb/image_raw` | Image | Main RGB feed (separate USB camera). |
| `/depth_cam/depth/image_raw` | Image | Depth feed (Orbbec). |
| `/depth_cam/depth/camera_info` | CameraInfo | **TRANSIENT_LOCAL (latched)** — subscribe with matching QoS (Round 17 PP.3). |
| `/depth_cam/rgb/camera_info` | CameraInfo | RGB intrinsics. |
| `/custom_sortingv5/image_result` | Image | Annotated viewer frame (overlay composited). |
| `/custom_sortingv5/status` | String (JSON) | 5 s heartbeat the UI polls (fps, depth, plane, last_calibrate, class_names, unmapped_count, ...). |
| `/calibration/image_result` | Image | Vendor calibration preview (tag + workspace rect). Position-tab preview subscribes here. |
| `/lab_manager/image_result` | Image (mono8) | Vendor LAB mask preview. Color-tab preview subscribes here (after ENTER). |

TF: static `depth_cam_link → depth_cam_color_frame` (looked up by
`_lookup_depth_color_tf`, folded into `hand2cam`).

## ROS services

**v5 node `/custom_sortingv5/*`:** `enter`, `exit`, `enable_sorting` (SetBool),
`set_target` (SetStringBool), `recalibrate`, `run_calibration` (drives the
vendor Position flow), `depth_plane_refit` (Trigger; returns reason in
`message`), `load_engine`, `reload_engine`, `save_profile`, `load_profile`,
`save_as_default`, `apply_and_persist`, `save_yolo_config`, `test_grip`,
`init_finish`.

**Vendor calibration `/calibration/*`:** `enter`, `start`, `exit` (Trigger) +
publishes `/calibration/finish` (Bool) when PnP solves.

**Vendor LAB `/lab_manager/*`:** `enter`, `exit`, `save_to_disk` (Trigger);
`get_range` (GetRange), `change_range` (ChangeRange), `stash_range`
(StashRange), `get_all_color_name` (GetAllColorName). Note: the vendor
`enter` callback ALWAYS returns success=True — a UI "enter failed" means the
client's service discovery timed out, not the node (fixed Round 17 OO).

## Config-file schemas

**transform.yaml** (written by Position + Depth calibration):
- `extristric`: 4×3 — [tvec(3), R row0, R row1, R row2] camera extrinsic.
- `corners`: 5×3 — 4 workspace corners + centre, world frame.
- `white_area_pose_cam` / `white_area_pose_world`: 4×4 mat poses.
- `plane`: [a,b,c,d] for ax+by+cz+d=0 (depth tab / plane refit; optional).

**lab_config.yaml** (vendor lab_manager):
- `/**: ros__parameters: color_range_list: {<color>: {min:[L,A,B], max:[L,A,B]}}}`
  — LAB space, 0–255. Default colors: red/green/blue/black/white/yellow/tennis.

**calibration.yaml**: `kinematics: {offset:[x,y,z], scale:[x,y,z]}` and
`pixel: {offset, scale}` — per-axis corrections applied in
`get_object_world_position` / `_apply_kinematics_calibration`.

**default.yaml** (`~/jetarm_v5_profiles/default.yaml`): all tunables under
`/**: ros__parameters:`. Key groups: motion/speed, grip, detection lock
gating (`lock_distance_thresh`, `count_still_threshold`, `count_move_threshold`,
`detection_avg_frames`), depth (`use_depth_for_z`, `depth_window_px`,
`depth_max_z_m`, `overlay_depth_view`), calibration overlay, `place_positions`
(JSON: per-class drop zones), `grasp_strength`, YOLO knobs, `engine_path`.

## Logs

- Node mirrors every `_stage(tag, msg)` to `~/jetarm_v5/logs/v5_session_*.log`;
  UI mirrors `_ui_log` to `v5_ui_*.log`. Uncaught exceptions appended via
  excepthook. Last 20 sessions kept.
- `_stage` format: `[v4][<tag>] <msg>`. Useful tags to grep: `sorting-loop`
  (incl. TRACK / LOCKED / FIRING TRANSPORT), `transport`, `calibrate`,
  `roi`, `init`, `depth`, `engine-load`.
- `JETARM_V5_DEBUG=1` enables verbose `_dbg` (per-frame).
- **Shipping logs:** UI "PUSH LOGS" button → `tools/push_logs.sh` copies the
  most recent sessions into `logs/` and pushes to the **`jetarm-logs`**
  branch. Needs device push credentials (HTTPS PAT or SSH) — see
  UPDATE_JETARM.md "Pushing logs".

## Vendor source — `full jetarm source for context src/`

The real Hiwonder JetArm source, for reference (do NOT edit; mirror its
behaviour). Key files:

| File | Use for |
|---|---|
| `app/app/calibration.py` | Position/AprilTag calibration backend (the `/calibration/*` node v5 drives). |
| `app/app/lab_manager.py` | LAB color threshold backend (the `/lab_manager/*` node). |
| `app/app/object_sorting.py` | The vendor color-based sorter — the reference for v5's `sorting_loop` / pick / place logic. |
| `app/app/shape_recognition.py` | Depth-based recognition pipeline (plane → world XYZ). |
| `app/app/utils/search_plane.py` | `SearchPlane` RANSAC plane fit (v5 imports this). |
| `app/app/utils/utils.py` | `calculate_world_position`, `get_plane_values`, `find_depth_range`, `convert_depth_to_camera_coords`. |
| `app/app/utils/common.py` OR `driver/sdk/sdk/common.py` | `pixels_to_world`, `world_to_pixels`, `xyz_euler_to_mat`, `extristric_plane_shift`, yaml helpers. |
| `app/app/utils/calculate_grasp_yaw_by_depth.py` | Depth-informed grasp yaw. |
| `bringup/launch/bringup.launch.py` | Shows `rosbridge_server` + `web_video_server` — proof the vendor GUI is a web app NOT in this repo. |
