# 06 — Plans: Done & Unfinished (+ memory dump)

Status legend: **[DONE]** shipped & code-verified · **[HW?]** shipped but
unverified on the physical arm · **[PARTIAL]** partially implemented ·
**[OPEN]** not done / needs decision.

The authoritative, blow-by-blow plan lives in the maintainer's plan file
(`~/.claude/plans/i-unplugged-...md`). This is the portable digest.

---

## Roadmap by round (with commit SHAs)

### Rounds 1–11 — building v5 (pre-handover)
- **[DONE]** YOLO-only detection, TensorRT engine hot-swap (`InferenceWorker`).
- **[DONE]** Tuner UI tabs (Speed/Grip/Detection/Model/Places/Toggles/Profiles).
- **[DONE]** Per-class place targets; buffered Model-config tab.
- **[DONE]** Calibration overlay (workspace rect, axes, per-object aim X).
- **[DONE]** Device-only old-version cleanup (`uninstall_others.sh`); repo
  keeps v2/v4/v4.1.
- **[DONE]** Depth-aware Z (Round 11): `_depth_at`, depth subscription, sanity
  gate.
- **[PARTIAL]** Beta force grip wired (`compliance_grasp`) — see UNFINISHED.

### Round 12 — `a2e26c5`
- **[DONE]** Overlay triangle fix (full-pose corner build + NaN/off-screen
  guard).
- **[DONE]** depth↔color TF lookup; RANSAC plane fit via `SearchPlane`;
  `plane` persisted in transform.yaml; depth heatmap overlay toggle.
- **[DONE]** Session logging (`_stage` → `~/jetarm_v5/logs/`), excepthook,
  `logs/` folder, `tools/push_logs.sh`, PUSH LOGS button.
- **[SUPERSEDED]** Inline AprilTag calibrate fallback — later deleted in R15.

### Round 13 — `f9f87dd`
- **[DONE]** PUSH LOGS repo detection via realpath walk-up; self-locating
  `push_logs.sh`.

### Round 14 — `13d1dec`
- **[SUPERSEDED]** In-node LAB color services + world-position scale/offset.
  The LAB part was removed in R15 (now the vendor lab_manager). The
  calibration.yaml scale/offset reading remains.

### Round 15 — `8223cfd` (+ hotfix `ab31b2a`)
- **[DONE]** Deleted v5 inline AprilTag calibrate + in-node LAB surface
  (vendor parity).
- **[DONE]** Calibrate tab split into **Position / Color / Depth**, each
  driving the vendor node; only-own-YAML writes.
- **[DONE]** `lab_manager` included in the launch; `launch_v5.sh` soft-waits
  for depth.
- **[DONE]** Hotfix: removed stray `calibrate_tab` ref that crashed the UI.

### Round 16 — `432fca1`
- **[DONE]** Place-zone defaults for the real class names (scaff →
  hazardous-waste, each cube → own grid spot) in `DEFAULT_PLACE_POSITIONS`
  and `default.yaml`.
- **[HW?]** Pick-trigger tuning + throttled diagnostics (later refined in R17).
- **[HW?]** Live previews (`_make_image_preview`) on Position/Color tabs.
- **[DONE]** Pop-out calibration window + `tools/calibration_tools.sh`.
- **[DONE]** Logs → dedicated `jetarm-logs` branch; stderr surfaced.
- **[DONE]** Removed stale `white_area_*`, dt_apriltags install step,
  "inline" comments.

### Round 17 — `1aea062`
- **[HW?]** **Lock by LABEL not instance index** — the real fix for "blue
  line but never grabs". Restored vendor-exact thresholds
  (`lock_distance_thresh` 0.005, `count_still_threshold` 10,
  `count_move_threshold` 10, `detection_avg_frames` 1); removed the R16
  `lock_timeout_frames` band-aid. Added LOCKED/FIRING TRANSPORT logs.
- **[HW?]** Color ENTER fix: service-discovery timeout 1→5 s;
  `_trigger_with_msg` (no wait_for_service, returns message).
- **[HW?]** Depth plane refit: TRANSIENT_LOCAL QoS for depth camera_info;
  `_fit_table_plane` returns `(plane, reason)`; UI shows the reason.

### Round 18 — this handover
- **[DONE]** `v5 full context/` folder (this).

---

## UNFINISHED / OPEN (read this if you're continuing the work)

1. **[HW?] Pick firing end-to-end.** The label-lock fix (R17) is the
   strongest candidate to make sorting work, but it was never run on the
   arm. First test: enable sorting on one cube, watch for
   `TRACK→LOCKED→FIRING TRANSPORT`. If those log but the arm doesn't move,
   the bug is downstream in `transport_thread`/`_do_pick`:
   - IK reachability of the computed world XYZ.
   - `_apply_kinematics_calibration` offsets/scale from `calibration.yaml`.
   - `_apply_world_offsets` (grip_offset_x/y, workspace_scale) — confirm
     they're at defaults (0,0,1.0) so they don't shove the target off.
   - `compliance_grasp` aborting early.
   Compare line-for-line with vendor `object_sorting.py` pick path.

2. **[HW?] Color ENTER + mask preview.** Verify ENTER now returns OK and the
   `/lab_manager/image_result` preview renders (needs `python3-pil.imagetk`).

3. **[HW?] Depth plane refit.** Verify it succeeds or returns a specific
   reason (e.g. `no_depth_caminfo_yet` should now be fixed by the QoS change).

4. **[OPEN] Live previews dependency.** `python3-pil.imagetk` must be
   installed on the device or previews show a hint label instead of video.
   Consider adding it to `install.sh` (guarded) if the user wants.

5. **[OPEN] Logs reaching GitHub.** PUSH LOGS pushes to `jetarm-logs`, but
   the device clone likely has no push credentials (it can pull over HTTPS,
   not push). One-time fix needed: a PAT credential helper or an SSH remote.
   Until then, logs only live in the device working tree + are pasted to the
   session manually. (This is why `logs/` on GitHub is still near-empty.)

6. **[PARTIAL] Beta force-grip closed loop.** `compliance_grasp` returns
   `gripped/closed/overheat/no_feedback`, but the intended behaviour — **fail
   the pick (don't proceed to place an empty gripper) when contact wasn't
   detected** — and **per-class strength tuning + live grip telemetry** from
   the original plan may not be fully wired into `_do_pick`/`transport_thread`.
   Verify whether a failed grasp still proceeds to "place". Hard constraint:
   the servos can't report load/current, so this is contact-stop + temp
   cutoff only, never true force feedback.

7. **[OPEN] Place accuracy.** Depends entirely on Position calibration
   quality. If cubes land off-zone, the fix is recalibration, not code —
   confirm the yellow aim-X sits on the cube first.

---

## Memory dump — things not obvious from the code

### Hardware / environment quirks
- **Camera steal on double bringup:** opening the Orbbec device twice → one
  process loses the USB claim → camera dead. Subscribers are safe; only the
  `depth_camera` bringup opens it. If the camera dies after launch, suspect a
  race. Recover by restarting `start_app_node.service`.
- **Divergent-branch trap:** the device accumulates local `logs: session …`
  commits from PUSH LOGS, so `git pull` fails. Always `reset --hard
  origin/main`. (Documented in UPDATE_JETARM.md.)
- **dpkg interrupted:** a past `apt` left dpkg half-configured, which blocked
  `ros-humble-rqt-image-view` install. Fix: `sudo dpkg --configure -a` then
  retry apt.
- **Orbbec USB re-enumeration:** if depth never publishes, unplug/replug the
  Astra USB and restart the service.
- Device user is `ubuntu`; hardcoded paths use `/home/ubuntu/...`.
- The Jetson is an **Orin Nano**; engines are TensorRT, class names come from
  `model.names` (current engine: `scaff, light blue cube, dark blue cube,
  red cube, green cube`).

### User's stated preferences (honor these)
- **Vendor parity, no deviation.** Drive the vendor nodes; don't reinvent
  their calibration/color logic.
- All calibration in the tuner **tabs**, AND able to **pop out** into a
  separate window on the same domain.
- **Sensible fixed** place-zone map (scaff→hazardous-waste, colors→own
  spots) — not random per launch.
- Wants logs/errors pushed to the repo for cross-session context.
- Wants changes committed + pushed to `main` and the feature branch when done.

### Dead-ends already ruled out (don't repeat)
- **No vendor GUI exists in this source** — it's a web app (rosbridge +
  web_video_server) in Hiwonder's image. Don't search for an html/qml file
  to copy; there isn't one.
- **Servos can't report load/current** — confirmed at the serial protocol
  (only position/voltage/temperature/torque-on-off). No software-only true
  force grip is possible.
- **Inline AprilTag calibration in v5** was tried and removed (R15) — the
  vendor node is the path.
- **Looser lock thresholds** (R16) did NOT fix sorting — the real bug was
  index-based locking (fixed structurally in R17 by matching on label).
