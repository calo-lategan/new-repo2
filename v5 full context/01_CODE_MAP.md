# 01 — Code Map

## Repo tree (top level)

```
new-repo2/
├── custom_sortingv5/          ★ ACTIVE — the v5 stack (node, UI, launch, scripts)
├── tools/                     ★ ACTIVE — push_logs.sh, calibration_tools.sh
├── full jetarm source for context src/   ★ REFERENCE — the vendor JetArm source
├── v5 full context/           ← this handover folder
├── logs/                      session logs land here when pushed (see jetarm-logs branch)
├── UPDATE_JETARM.md           device update + troubleshooting (authoritative)
├── README.md
├── custom_sortingv4.1/        ARCHIVE — kept in repo, removed on device by installer
├── v4 custom sorting/         ARCHIVE
├── v2 custom sorting/         ARCHIVE
└── example pi scripts/        ARCHIVE / misc
```

Only `custom_sortingv5/`, `tools/`, and the vendor `full jetarm source for
context src/` matter for active work. v2/v4/v4.1 are kept for history but
the installer uninstalls them from the device.

## `custom_sortingv5/` file-by-file

| File | Lines | Purpose |
|---|---|---|
| `custom_sortingv5.py` | 3710 | The ROS 2 sorting node. Cameras always-on, YOLO-only detection, hot-swap engine, pick/place state machine, vendor-calibration clients. |
| `tune_uiv5.py` | 2816 | Tkinter tuner UI. Tabs: Speed / Grip / Detection / Places / Position / Color / Depth / Toggles / Profiles. Talks to the node + vendor nodes over ROS services. |
| `custom_sorting_nodev5.launch.py` | 270 | ROS 2 launch. Includes vendor `depth_camera`, `calibration_node`, **and** `lab_manager` (Round 15). Pins ROS_DOMAIN_ID via the wrapper. |
| `launch_v5.sh` | 578 | One-click bash launcher. Stops/disables the factory service, waits for RGB **and** depth topics, pins ROS_DOMAIN_ID=0, then `ros2 launch`. |
| `install.sh` | 715 | Idempotent installer: clone → symlink sources into `~/ros2_ws/src/app` → patch setup.py → seed profiles → desktop shortcut → `colcon build --symlink-install`. |
| `image_view_chain.sh` | 161 | rqt → image_view → browser viewer chain, IP autodetected. |
| `uninstall_others.sh` | 84 | Removes v2/v4/v4.1 from the device. |
| `re-enable-factory.sh` | 24 | Restores Hiwonder's factory app. |
| `match_launcher_env.sh` | 48 | Helper to match the launcher's env. |
| `tools/diag.sh` | — | On-device diagnostics. |
| `profiles/default.yaml` | — | **The boot source of truth.** Per-tab "Save & Apply" merges here. Seeded to `~/jetarm_v5_profiles/default.yaml` at install. |
| `profiles/yolo.yaml` | — | Deprecated/ignored sample. |
| `INSTALL.md` | — | Full device install guide. |

## `custom_sortingv5.py` — structure (by line)

Module-level (1–230):
- Session logging: `_open_session_log` (82), `_stage` (109 — the primary
  diagnostic; mirrors to `~/jetarm_v5/logs/`), `_install_excepthook` (134),
  `_dbg` (157), `_set_debug` (165).

`InferenceWorker(threading.Thread)` (231–486): TensorRT/YOLO worker.
- `set_yolo_knobs` (274), `class_names` (288), `pause/resume` (297/300),
  `submit`/`latest` (306/311), `request_engine_swap` (320), `_load` (350),
  `run` (402). Hot-swaps engines between frames.

`MotionController` (488–688): all arm motion.
- `goto_pose` (546), `set_gripper` (579), `set_wrist` (583),
  **`compliance_grasp` (587)** — the force-limited grasp (contact-stop +
  temp cutoff; servos can't report load, see 03).

`ObjectSortingNodeV5(Node)` (718–3678): the node. Grouped by concern:
- **Lifecycle/params**: `__init__` (892), `_apply_default_profile_seed`
  (1238), `_declare_tunables` (1268), `_on_param_change` (1333), `p` (1404),
  `_init_state` (1440), `_heartbeat` (1475 — publishes JSON status), `_startup`
  (1585).
- **ROI/world setup**: `get_roi` (1632 — loads transform.yaml incl. `plane`),
  `go_home` (1619).
- **Services**: `enter` (1717), `exit` (1743), `enable_sorting` (1760),
  `set_target` (1781), `recalibrate` (1794), `run_calibration` (1798),
  `_depth_plane_refit_srv` (1831), `test_grip` (2031), `apply_and_persist`
  (2140), `save_yolo_config` (2173), `load_engine` (2253), `reload_engine`
  (2265), `save_profile`/`load_profile` (2288/2305), `save_as_default` (2341).
- **Calibration (vendor clients)**: `_on_calib_finish` (1790), `_call_trigger`
  (1817), `_run_vendor_calibration` (1866 — enter→start→finish→exit),
  `_flash_overlay` (1931), `_fit_table_plane` (1951 — vendor SearchPlane,
  returns (plane, reason)).
- **World-position math**: `_pixel_to_world` (2357), `_calibration_cfg` (2452 —
  reads calibration.yaml), `_apply_world_offsets` (2466), `get_object_world_position`
  (2503), `calculate_pick_grasp_yaw` (2586), `_select_gripper_yaw` (2597),
  `calculate_place_grasp_yaw` (2646), `_apply_kinematics_calibration` (2657).
- **Pick/place state machine**:
  - `sorting_loop` (2904) — detect, lock onto target, fire transport.
  - `transport_thread` (2865) — waits on `start_transport`, runs the pick.
  - `_do_pick` (2672) — the grasp sequence (uses compliance_grasp).
  - `_do_place` (2811) + `_resolve_place_position` (2794) +
    `_grasp_strength_for` (2782) + `_per_target_overrides` (2663).
- **Camera/depth**: `image_callback` (3317), `camera_info_callback` (3152),
  `_depth_cam_info_callback` (3164 — TRANSIENT_LOCAL QoS), `_lookup_depth_color_tf`
  (3169), `_depth_callback` (3221), `_depth_at` (3257 — above-plane height gating).
- **Overlay/viewer**: `_publish_image` (3091), `_draw_overlay` (3341),
  `_draw_calibration_overlay` (3376), `_project_world_to_pixel` (3592),
  `_raw_republish_tick` (3623).
- `main` (3679).

## `tune_uiv5.py` — structure

- Module: session logging (`_ui_log`), excepthook, param tables
  (FLOAT/INT/MODEL/CALIB/CALIB_DEPTH/BOOL params), `DEFAULT_PLACE_POSITIONS`.
- `TunerClient(Node)` (~255): every ROS client. Helpers `_trigger`,
  **`_trigger_with_msg`** (Round 17 — skips wait_for_service, returns
  (ok, message)), `_set_string_bool`, `lab_*` (enter/exit/get_range/
  change_range/stash_range/save), `trigger_service`.
- `TunerUI` (~494): the window.
  - `__init__` (takes `calib_only` for pop-out mode), `_build`,
    `_build_calib_only`.
  - `_poll_node_status` — reads the heartbeat JSON, updates all status panels.
  - Tab builders: `_build_model_section`, `_build_places_tab`,
    `_build_position_tab`, `_build_color_tab`, `_build_depth_tab`,
    `_build_profiles_tab`.
  - `_make_image_preview` (live ROS image → Tk via Pillow),
    `_spawn_calibration_window` (pop-out a separate window, same domain).
  - Handlers: `_on_calibrate`, `_on_plane_refit`, `_on_color_enter/exit/
    stash/save`, `_on_push_logs`.
  - `main` — argparse `--node-name`, `--calib-window`.

## End-to-end data flow

```
/depth_cam/rgb/image_raw
        │  image_callback()
        ▼
   InferenceWorker.submit(frame)   ──► YOLO (TensorRT)  ──► InferenceWorker.latest()
        │
        ▼
   sorting_loop()                 (gated on enable_sorting)
     • _detections_from_results → boxes + labels
     • position_reorder (stable-ish ids)
     • LOCK by LABEL (Round 17): match target[0]==label, pick nearest pixel
     • get_object_world_position → world XYZ (+ offsets, + depth via _depth_at)
     • count_still ≥ threshold  ──► transport_info = [pos, yaw, target]
                                    start_transport = True
        │
        ▼
   transport_thread()  (waits on start_transport)
     • _do_pick(pos, pitch, yaw, label)
         – goto pre-pose → descend → compliance_grasp() → lift
     • _do_place(label)
         – _resolve_place_position(label) → per-class zone → drop
     • go_home()
```

Overlay: `_raw_republish_tick` composites the latest YOLO + lock overlay onto
the raw frame and republishes to `/custom_sortingv5/image_result` for the
viewer, independent of the sorting state.
