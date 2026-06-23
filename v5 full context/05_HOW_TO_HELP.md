# 05 — How To Help (operating manual for this session)

## Update the device

```bash
git -C ~/jetarm_v5_src fetch origin
git -C ~/jetarm_v5_src reset --hard origin/main      # NOT `git pull`
bash ~/jetarm_v5_src/custom_sortingv5/install.sh      # only if launch/setup.py changed
~/jetarm_v5/launch_v5.sh
```

- Use **`reset --hard`**, never `git pull`. The device accumulates local
  log commits (from PUSH LOGS) that make `pull` fail with "divergent
  branches". `reset --hard origin/main` always wins; logs are regenerable.
- Pure Python edits (node/UI) don't need `install.sh` — the sources are
  symlinked, so `reset --hard` + relaunch is enough. Re-run `install.sh`
  only when launch files, setup.py entry points, or dependencies changed.
- First time only, for live previews: `sudo apt install -y python3-pil.imagetk`.

## Read the logs (your main debugging surface)

- Node writes `~/jetarm_v5/logs/v5_session_<pid>_<ts>.log`. Launch with
  `JETARM_V5_DEBUG=1 ~/jetarm_v5/launch_v5.sh` for verbose per-frame `_dbg`.
- Grep tags (`[v4][<tag>]`): `sorting-loop`, `transport`, `calibrate`,
  `roi`, `init`, `depth`, `engine-load`.
- The sorting trigger trace (Round 17), in order:
  - `TRACK <label> e_dist=… still=N/10 move=N/10` (~1 Hz while tracking)
  - `LOCKED on <label> pixel=… world=…` (target acquired)
  - `FIRING TRANSPORT label=… pos=… yaw=…` (pick triggered)
  If you see TRACK but `still` never reaches 10 → the object/centroid is too
  jittery or `lock_distance_thresh` too tight. If FIRING TRANSPORT logs but
  the arm doesn't move → it's downstream (`_do_pick` IK / kinematics offsets
  / compliance_grasp), not the detection lock.
- To get logs onto GitHub: tuner UI **PUSH LOGS** (lands on the
  `jetarm-logs` branch) or `bash ~/jetarm_v5_src/tools/push_logs.sh`. If it
  fails, the device lacks push creds — see UPDATE_JETARM.md.

## Verify a change end-to-end

| Feature | Check |
|---|---|
| Sorting | Enable sorting on ONE cube → logs show TRACK→LOCKED→FIRING TRANSPORT → arm picks & places in its zone. |
| Position calib | Position tab → CALIBRATE POSITION → "CALIBRATED (vendor)"; `transform.yaml` refreshed; yellow aim-X sits on the cube. |
| Color calib | Color tab → ENTER → "LAB: enter OK", mask preview renders; drag slider → mask updates; STASH → SAVE → `lab_config.yaml` updated. |
| Depth calib | Depth tab → CALIBRATE DEPTH → "Plane refit OK" + `plane:` in transform.yaml, OR an explicit `PLANE REFIT FAILED: <reason>`. |
| Places | Heartbeat `unmapped_count: 0`; Places tab pre-filled for all classes. |

## ROS quick-checks (run with ROS_DOMAIN_ID=0)

```bash
ros2 node list | grep -E "custom_sortingv5|calibration|lab_manager"
ros2 topic hz /depth_cam/rgb/image_raw      # ~30 Hz
ros2 topic hz /depth_cam/depth/image_raw    # >0 Hz
ros2 service list | grep -E "lab_manager|calibration|custom_sortingv5"
```

## Git conventions

- Develop on `claude/optimize-jetarm-performance-2HqF5`; push to it **and**
  `main` (the device updates from `main`).
- Session logs go to the `jetarm-logs` branch only.
- Commit message trailers (this project's convention):
  `Co-Authored-By: Claude <noreply@anthropic.com>` + a `Claude-Session:` line.
- Never put a model identifier in commits, PR text, or code.
- Compile-check before committing:
  `python3 -m py_compile custom_sortingv5/custom_sortingv5.py custom_sortingv5/tune_uiv5.py`
  and `bash -n` the shell scripts; `python3 -c "import yaml; yaml.safe_load(open('custom_sortingv5/profiles/default.yaml'))"`.

## Guardrails

- **Don't reimplement vendor calibration.** Drive `/calibration/*`,
  `/lab_manager/*`, and `SearchPlane`. (See `03_ARCHITECTURE.md`.)
- **Don't add a second camera consumer** that opens the Orbbec device — it
  can steal the camera. Subscribing is fine; opening is not.
- **default.yaml overrides node defaults.** If you change a tunable's default
  in `TUNABLE_PARAMS`, change it in `profiles/default.yaml` too, or the YAML
  wins on the device.
- Match the surrounding code style; the node uses `_stage(tag, msg)` for all
  diagnostics (it auto-mirrors to the session log).
