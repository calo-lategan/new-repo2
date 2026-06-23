# 04 — History & Current State

This is the condensed changelog. The full per-item DONE/PARTIAL/UNFINISHED
breakdown is in `06_PLANS_DONE_AND_UNFINISHED.md`.

## Round-by-round (recent rounds have commit SHAs)

- **Rounds 1–11 (pre-handover)** — built v5 from v4.1: YOLO-only detection,
  hot-swap engine, per-class place targets, buffered Model tab, tuner UI
  tabs, the calibration overlay, depth-aware Z (Round 11). Earlier rounds
  also: device-only old-version cleanup, the beta force grip wiring.
- **Round 12 (`a2e26c5`)** — overlay triangle fix (full-pose corner build +
  NaN guard); diagnose + inline AprilTag fallback; depth↔color TF; RANSAC
  plane fit; depth heatmap overlay; **session logging + `logs/` + push_logs**.
- **Round 13 (`f9f87dd`)** — fix PUSH LOGS repo detection (realpath walk-up;
  self-locating script).
- **Round 14 (`13d1dec`)** — (superseded) in-node LAB color services + world
  position scale/offset work.
- **Round 15 (`8223cfd`)** — **vendor parity refactor.** Deleted the v5
  inline AprilTag calibrate + the in-node LAB service surface. Split the
  Calibrate tab into **Position / Color / Depth**, each driving the vendor
  node. Included `lab_manager` in the launch. launch_v5 waits for depth.
  - **Hotfix (`ab31b2a`)** — stray `calibrate_tab` reference crashed the UI.
- **Round 16 (`432fca1`)** — place-zone defaults for the real class names;
  pick-trigger tuning + diagnostics; **live previews** (Position/Color) +
  **pop-out window** + `calibration_tools.sh`; logs → dedicated `jetarm-logs`
  branch; removed stale `white_area_*`, dt_apriltags install step.
- **Round 17 (`1aea062`)** — **make sorting fire:** lock by LABEL not index
  (the real "blue line but no grab" cause); restored vendor-exact thresholds;
  added LOCKED/FIRING TRANSPORT logs. **Color ENTER fix:** longer service
  discovery timeout + `_trigger_with_msg`. **Plane refit fix:** TRANSIENT_LOCAL
  QoS for depth camera_info + `(plane, reason)` surfaced in the UI.
- **Round 18 (this handover)** — the `v5 full context/` folder.

## Current state

### Working (verified in code / earlier sessions)
- Camera always-on; YOLO detection + hot-swap engine.
- Position calibration via the vendor node (`CALIBRATE button → OK
  source=vendor` seen in logs).
- Depth→color TF resolves; `transform.yaml` loads incl. `plane`.
- Workspace overlay (rectangle/axes/aim markers), depth heatmap toggle.
- Profiles, per-tab Save & Apply, default.yaml seeding.
- Session logging to `~/jetarm_v5/logs/`.

### Shipped but UNVERIFIED on hardware (this was a remote session)
- **Pick actually firing** end-to-end after the Round 17 label-lock fix.
  Watch for `FIRING TRANSPORT` in the logs and physical arm motion.
- **Color ENTER + mask preview** after the Round 17 timeout fix.
- **Depth plane refit** after the QoS + reason fix.
- **Live previews** require `python3-pil.imagetk` installed on the device.
- **Logs reaching GitHub** depend on the device having push credentials
  (jetarm-logs branch) — may still need a PAT/SSH setup.

### Open / next suspects
- If `FIRING TRANSPORT` logs but the arm doesn't move: look downstream in
  `_do_pick` (IK reachability), `_apply_kinematics_calibration` (offsets),
  and `compliance_grasp` (could be aborting). Compare against vendor
  `object_sorting.py` pick path.
- Beta force-grip closed-loop (fail-pick-on-no-contact + per-class strength)
  may be incomplete — see `06`.
- Place accuracy depends on calibration quality; if cubes land off-zone,
  recalibrate Position then verify the yellow aim-X sits on the cube.
