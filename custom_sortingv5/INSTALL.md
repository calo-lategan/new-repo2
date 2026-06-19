# Custom Sorting v5 — install on the Hiwonder JetArm (Jetson Orin Nano)

v5 is the single, current version of the custom sorting stack — **one YOLO model for all detection** (no OpenCV/LAB color path), per-class place targets, a buffered Model-config tab, an opt-in force-limited grasp, and AprilTag calibration delegated to the vendor node. It replaces v2 / v4 / v4.1.

| File | What it is |
|---|---|
| `custom_sortingv5.py` | The sorting node. Cameras always-on, YOLO-only detection, hot-swap engine. |
| `tune_uiv5.py` | Tkinter tuner UI: Speed / Grip / Detection / Model / Places / Toggles / Profiles. |
| `custom_sorting_nodev5.launch.py` | The ROS 2 launch file (includes the vendor AprilTag calibration node). |
| `launch_v5.sh` | One-click bash launcher. Stops + disables the factory service first. |
| `image_view_chain.sh` | rqt → image_view → browser chain, IP autodetected. |
| `re-enable-factory.sh` | Opt-in: put Hiwonder's factory app back. |
| `install.sh` | One-paste idempotent installer. |
| `profiles/` (created at install) | `~/jetarm_v5_profiles/{default,yolo}.yaml` (`yolo.yaml` is written by the Model-tab SAVE). |

**v5 needs a YOLO model whose `model.names` include your objects** (e.g. cubes + scaff in one model). The class names drive the class filter, the per-class place targets, and per-class grip strength. Until you load such a model, v5 only detects whatever classes are in the currently-loaded engine.

---

## One-paste install

Open a terminal **inside the Hiwonder container** and run:

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/calo-lategan/new-repo2/main/custom_sortingv5/install.sh) --sudoers
```

This will:

1. Clone the repo into `~/jetarm_v5_src`
2. Symlink `custom_sortingv5.py` and `tune_uiv5.py` into `~/ros2_ws/src/app/app/`
3. Symlink `custom_sorting_nodev5.launch.py` into `~/ros2_ws/src/app/launch/`
4. Idempotently add the two `console_scripts` entries (`custom_sortingv5`, `tune_uiv5`) to `setup.py`
5. Seed `~/jetarm_v5_profiles/` with `default.yaml` + `yolo.yaml` (the Model-tab SAVE overwrites `yolo.yaml`)
6. Install `~/jetarm_v5/launch_v5.sh` + `image_view_chain.sh` + `re-enable-factory.sh`
7. Drop a **JetArm Sort v5** desktop shortcut on your Desktop and in the app menu
8. Run `colcon build --packages-select app --symlink-install`
9. (because of `--sudoers`) Install NOPASSWD rules so the launcher can `stop`/`disable`/`restart`/`start` `start_app_node.service` without prompting

Once it finishes, double-click **JetArm Sort v5** on your desktop.

---

## What launching does (in order)

1. **Stops + disables** `start_app_node.service` (idempotent — runs every launch). The Hiwonder factory app grabs the camera + serial port + runs its own pick/sort code, which conflicts with v5. We never want it running alongside us.
2. Waits for the Hiwonder container to be ready (`/opt/ros/humble/setup.bash` exists, `~/ros2_ws/` exists).
3. Sources ROS 2 humble + the local install tree.
4. Boots: the v5 sorting node, the tuner UI, **and `web_video_server`** (because the factory service we just disabled was the thing that used to start it — we now start it ourselves on port 8080).
5. After ~8 s the `image_view_chain.sh` fires automatically and opens `rqt_image_view` in its own terminator window (you can disable this with `image_view:=false`).

---

## Camera view (no IP lookup, no fights with Docker)

The tuner UI has a `Camera view` row with four big buttons. **A viewer auto-opens at launch**; these buttons replace/swap it:

- **Open rqt_image_view** (green) — Qt window with topic dropdown. Runs inside `terminator` with `QT_X11_NO_MITSHM=1`, `QT_QPA_PLATFORM=xcb`, and `LIBGL_ALWAYS_SOFTWARE=1` exported — that's the combination that makes rqt actually render inside the Hiwonder Docker container's forwarded X11. Output is also tee'd to `/tmp/jetarm_v5_rqt.log` so you can `cat` it if the window misbehaves.
- **Open image_view** (blue) — the C++ classic, same X11 env, output tee'd to `/tmp/jetarm_v5_image_view.log`.
- **Open browser** (purple) — opens `http://<jetson-ip>:8080/stream?topic=/custom_sortingv5/image_result&th=100`. IP detected via `hostname -I`. `web_video_server` is now started by our launch on port 8080 (it used to come from the factory service we disable), so this works standalone.
- **Close viewer** (red) — kills the currently tracked viewer subprocess.

The publisher uses default RELIABLE QoS with depth=10 — that matches what `image_view`, `rqt_image_view`, and `web_video_server` request by default in humble. (The earlier round briefly tried `sensor_data` (BEST_EFFORT) which turned out to be incompatible — those consumers demand RELIABLE.)

### Disable auto-popup on launch

One-shot (this run only):

```bash
~/jetarm_v5/launch_v5.sh image_view:=false
```

Persistent (every run; survives reboot):

```bash
echo 'JETARM_V5_IMAGE_VIEW=false' >> ~/.jetarm_v5.env
```

The launcher reads `~/.jetarm_v5.env` on every start (same file you use for `ROS_DOMAIN_ID`). The tuner UI buttons still work either way.

### Point the viewer at a different topic

```bash
~/jetarm_v5/launch_v5.sh image_view_topic:=/depth_cam/rgb/image_raw
```

(Or change the topic field in the tuner UI before clicking Open.)

---

## Debugging from a second terminal — match the launcher's ROS env

**The single most common reason for "blank viewer" / "topic not published yet" is `ROS_DOMAIN_ID` mismatch.** The launcher prints its domain in the banner:

```
[env] ROS_DOMAIN_ID=0    <-- match this in other terminals!
```

To probe the running stack from a second terminal, source the helper that ships with v5:

```bash
source ~/jetarm_v5/match_launcher_env.sh
```

That sets `ROS_DOMAIN_ID` to whatever the launcher used, sources ROS humble + the workspace + orbbec_ws, and prints the visible nodes so you can confirm discovery worked.

Override the domain by either:

- One-shot: `ROS_DOMAIN_ID=5 ~/jetarm_v5/launch_v5.sh` (then set the same `ROS_DOMAIN_ID=5` before sourcing `match_launcher_env.sh`)
- Persistent: put `ROS_DOMAIN_ID=5` in `~/.jetarm_v5.env` — both the launcher and `match_launcher_env.sh` source it.

## Debugging frame flow

If a viewer still shows a blank window AFTER matching the domain, the live frames may not be flowing. Diagnose:

```bash
# 1. Is the topic actually publishing?
ros2 topic hz /custom_sortingv5/image_result          # expect ~15-30 Hz

# 2. What does the first message look like?
ros2 topic echo --once /custom_sortingv5/image_result --no-arr | head -20
#    encoding: bgr8 / height: 480 / width: 640 / step: 1920

# 3. Direct rqt_image_view test with X11 env + raw transport. Without
#    image_transport:=raw, rqt negotiates compressed/theora/compressedDepth
#    plugins. They fail to encode the orbbec depth (16UC1) and the RGB
#    in mismatched directions, log ~30 Hz of errors, and eventually
#    crash the depth_cam component_container with an OpenCV OOM.
QT_X11_NO_MITSHM=1 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
    ros2 run rqt_image_view rqt_image_view /custom_sortingv5/image_result \
        --ros-args -p image_transport:=raw

# 4. web_video_server in browser (ros_compressed = streams the node's
#    pre-encoded JPEG sibling topic, ~10x less bandwidth than raw; tune
#    quality with the publish_jpeg_quality slider in the Vision tab)
firefox "http://$(hostname -I | awk '{print $1}'):8080/stream?topic=/custom_sortingv5/image_result&type=ros_compressed"

# 5. Confirm factory service is stopped + disabled
systemctl is-active   start_app_node.service           # expect "inactive"
systemctl is-enabled  start_app_node.service           # expect "disabled"

# 6. Confirm web_video_server is listening (we start it now)
ss -tlnp | grep 8080
curl -sI http://localhost:8080/ | head -1              # expect "HTTP/1.1 200"

# 7. Confirm RELIABLE QoS on the publisher (matches image_view / web_video_server)
ros2 topic info /custom_sortingv5/image_result --verbose
#    Look for: Reliability: RELIABLE, History (Depth): 10

# 8. If rqt window is blank/black, dump its log
cat /tmp/jetarm_v5_rqt.log

# 9. Confirm subscribers are connected. The heartbeat now reports result_subs=N.
#    With one rqt open: result_subs=1. With rqt + browser: result_subs>=2.
#    If result_subs=0 while a viewer is open, you're in a different ROS_DOMAIN
#    or DDS discovery is broken - see "Debugging from a second terminal" above.
```

### depth_cam crash recovery

If you see `[component_container] terminate called` + `OpenCV ... Failed to allocate ... bytes` in the launcher, the orbbec driver crashed. v5's heartbeat will print `cam_fps=0.0` from then on. To recover without restarting everything:

```bash
pkill -f component_container
~/jetarm_v5/launch_v5.sh
```

The most common trigger was the image_transport plugin storm; v5 round 6+ forces `image_transport:=raw` on every viewer so it shouldn't happen anymore. If it does, paste the lines leading up to the crash.

On the node side, the first time `_publish_image` succeeds it prints:

```
[v5][publish] first frame published: shape=(480, 640, 3) step=1920 contig=True topic=/custom_sortingv5/image_result
```

If you see that line, the publish path is healthy and any remaining blank window is on the viewer side (re-check X11 env / browser URL throttle).

For verbose stage logs:

```bash
JETARM_V5_DEBUG=1 ~/jetarm_v5/launch_v5.sh
```

---

## The Model tab (buffered — nothing applies until SAVE)

Everything that configures the YOLO model lives on one **Model** tab:
the engine picker, the YOLO knobs, and the per-class enable checkboxes.
**Edits stay local until you press SAVE MODEL CONFIG** — the tab shows
`Model *` while you have unsaved changes and prompts if you switch away.

SAVE writes `~/jetarm_v5_profiles/yolo.yaml` and **hot-applies atomically**:
the engine swaps between inference frames (no app restart), and conf / iou /
max_det / enabled-classes land together. On boot the node auto-loads
`yolo.yaml`.

| Knob | Default | Effect |
|---|---|---|
| `engine_path` | best_scaff2.engine | TensorRT `.engine` (or `.pt`). Pick from the list, Browse, or type a path. |
| `yolo_conf_thresh` | 0.25 | Confidence threshold. Higher = stricter. |
| `yolo_iou_thresh` | 0.7 | NMS IoU. |
| `yolo_max_det` | 100 | Max detections per frame. |
| `inference_max_hz` | 0 | Cap inference rate (0 = uncapped). |
| Enabled classes | all | Tick which `model.names` classes to detect (none ticked = all). Scrollable + filter box + All/None/Invert for big models. |

`yolo_imgsz` is **not** tunable — TensorRT engines bake the input size in at
export; re-export from Ultralytics and pick the new `.engine` here.

## The Places tab (per-class targets)

Rows auto-populate from the loaded model's class names. For each class set:

- **x / y / z** — the world drop point for that object.
- **grip** — the per-class **max hold strength** (max close pulse) used by
  the force-limited BETA grasp, so fragile cubes get a gentle cap and scaff
  gets a firm one.

Save a single row, or **Save all rows**. A class with no place target is
**refused at drop time** (no random drops) and its count is badged in the
perf line (`unmapped=N`).

## Grip: standard by default, force-limited is opt-in BETA

The default grasp is the standard one-shot close-to-pulse
(hover → align → descend → close → settle → lift). Flip
**`compliance_grasp_enabled`** (Toggles tab) to switch to the BETA
**force-limited "close until contact"** grasp.

The hardware exposes **no servo load/current** (only position / voltage /
temperature), so this is an honest *contact-stop*, not a force sensor: the
jaws close in small steps and stop the moment the position stalls against
the object. The slow small-step approach **is** the force limit — it can't
slam — and per-class `grip` strength caps how hard it can ever squeeze.
A temperature cutoff (`grasp_max_temp`) protects the gripper servo.

| Knob | Default | Effect |
|---|---|---|
| `grasp_step_pulse` | 15 | Pulses per step. Smaller = gentler contact. |
| `grasp_step_dwell` | 0.10 | Sec per step (≥ one 50 Hz driver cycle so readback is fresh). |
| `grasp_stall_pulse` | 8 | Advance under this per step = contact detected. |
| `grasp_timeout` | 2.0 | Sec failsafe budget. |
| `grasp_max_temp` | 65 | °C — stop to protect the gripper servo. |
| `gripper_settle` / `grab_depth` | 0.5 / 0.02 | Settle before lift; descend below z so jaws wrap the body. |

Per-class overrides also work via `target_overrides` JSON, e.g.
`'{"scaff": {"grab_depth": 0.025}, "cube_red": {"motion_speed": 1.8}}'`.

### Closed-loop pick

With the BETA grasp enabled, an **empty close** (`closed` outcome — jaws
reached the cap without contacting anything) **fails the pick**, so the
transport opens the gripper, returns home, and the next detection cycle
re-locks the target naturally. No retry loop, no spam — paced by detection.
`overheat` and `aborted` bail the same way. The Grip tab shows the live
outcome and the perf line badges `unmapped=N` for classes with no place
target.

### Tuning per-class strength (the Test grip button)

Each row in the **Places** tab has a **Test grip** button next to Save:

1. Stop sorting first.
2. Pick a row (e.g. `cube_red`) and click **Test grip**. The arm moves to
   a safe test pose, opens the jaws, and dwells `test_grip_dwell` seconds
   (default 3).
3. **Place the object between the jaws** during the dwell.
4. The gripper closes until contact (or the per-class cap), holds briefly
   so you can eyeball it, then releases.
5. The Grip tab updates with `last grip [test]: cube_red gripped <pulse>p
   <temp>C <ms>ms`. If it's mushy: raise the strength entry, Test again.
   If the servo whines / temp climbs fast: lower it. **Save** the row when
   it's right.

This lets you tune the per-item cap interactively without firing a full
pick cycle and without ever dragging the YOLO model into it.

## Calibrate (AprilTag — same as the factory app)

The **CALIBRATE** button runs the **vendor AprilTag calibration node** (it's
launched alongside v5 and sits idle until triggered). It **stops sorting and
moves the arm** to the calibration pose, collects AprilTag readings, solves
the camera→world transform, and writes `transform.yaml` — the same file the
sorting node uses for its ROI / world projection. v5 rebuilds the ROI
automatically when it finishes.

Requires **AprilTags IDs 1 / 2 / 3 (fallback 100), 2.5 cm**, flat and fully
in the camera view. The button confirms before moving the arm. If the
calibration node isn't running, v5 logs and stays alive (no crash).

### Heartbeat mirror

### Heartbeat mirror

The node publishes its 5 s heartbeat as JSON on
`/custom_sortingv5/status`. The tuner UI subscribes and polls it once a
second: the **perf label** shows live `cam/pub fps`, AI state and
inference age; the **Engine label** tracks hot-swaps; and the
**RUNNING/STOPPED** status corrects itself if the node state changes
behind the UI's back (e.g. the heartbeat watchdog called `exit`). If the
node stops publishing for >12 s the perf label shows `NO HEARTBEAT`.

### New top-bar buttons

- **Pause camera** — drops our subscription to `/depth_cam/rgb/image_raw`. orbbec keeps publishing, we stop consuming. `cam_fps` heartbeat drops to 0. The raw-republish tick keeps emitting the last frame, so the viewer doesn't go blank.
- **Pause AI** — pauses the `InferenceWorker`. The camera path keeps running and publishing raw frames (no annotation overlay). Use this to A/B "camera-only fps" vs "camera + AI fps".

Heartbeat line now shows both: `cam=LIVE/PAUSED ai=LIVE/PAUSED cam_fps=N.N pub_fps=N.N`.

---

## Bringing back the factory app

The launcher disables `start_app_node.service` on every run. To go back to Hiwonder's factory bringup:

```bash
~/jetarm_v5/re-enable-factory.sh
```

This `unmask`s, `enable`s, and `start`s the service. Note that running `launch_v5.sh` again will re-disable it — that's intentional. To keep the factory app permanently, don't launch v5.

---

## Updating later (no rebuild)

The installer symlinks the sources and builds with `--symlink-install`, so pulling new code is enough:

```bash
git -C ~/jetarm_v5_src pull
~/jetarm_v5/launch_v5.sh
```

Only re-run `install.sh` if `setup.py` / `package.xml` / launch arguments changed upstream.

---

## Uninstall

```bash
rm -rf ~/jetarm_v5 ~/jetarm_v5_profiles ~/jetarm_v5_src \
       ~/ros2_ws/src/app/app/custom_sortingv5.py \
       ~/ros2_ws/src/app/app/tune_uiv5.py \
       ~/ros2_ws/src/app/launch/custom_sorting_nodev5.launch.py \
       ~/Desktop/jetarm-sort-v5.desktop \
       ~/.local/share/applications/jetarm-sort-v5.desktop
sudo rm -f /etc/sudoers.d/jetarm-v5
# remove the two console_scripts lines from ~/ros2_ws/src/app/setup.py
cd ~/ros2_ws && colcon build --packages-select app
~/jetarm_v5/re-enable-factory.sh   # if you want the Hiwonder factory app back
```
