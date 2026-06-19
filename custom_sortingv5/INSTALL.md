# Custom Sorting v5 — install on the Hiwonder JetArm (Jetson Orin Nano)

v5 is the stable cleanup of v4. Folder layout uses **`v5` for Python module names** (Python rejects a literal dot as a package separator) and **`v5` for everything else** — launcher script, launch file, desktop shortcut, the folder itself.

| File | What it is |
|---|---|
| `custom_sortingv5.py` | The sorting node. Cameras always-on, gated inference, hot-swap engine. |
| `tune_uiv5.py` | Tkinter tuner UI. Buttons to open rqt_image_view / image_view / browser. |
| `custom_sorting_nodev5.launch.py` | The ROS 2 launch file. |
| `launch_v5.sh` | One-click bash launcher. Stops + disables the factory service first. |
| `image_view_chain.sh` | rqt → image_view → browser chain, IP autodetected. |
| `re-enable-factory.sh` | Opt-in: put Hiwonder's factory app back. |
| `install.sh` | One-paste idempotent installer. |
| `profiles/` (created at install) | `~/jetarm_v5_profiles/{default,fast,precision}.yaml` |

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
5. Seed `~/jetarm_v5_profiles/` with the default + fast + precision profiles
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

## Performance + tunables (round 7)

### What runs where on the Orin Nano

| Step | Hardware |
|---|---|
| **YOLO TensorRT inference** | **GPU** (Ampere CUDA + FP16 Tensor cores). Confirmed by `[TRT] TensorRT-managed allocation` lines on every launch. |
| **rqt / image_view viewer paint** | **GPU** by default in round 7+. Earlier rounds forced software rendering via `LIBGL_ALWAYS_SOFTWARE=1` — removed. |
| OpenCV drawing (rectangle, putText, line) | CPU. ~1 ms/frame — not the bottleneck. |
| cv_bridge + DDS publish | CPU. ~1-2 ms/frame. |
| Orbbec USB capture | CPU. Driver-level, unavoidable. |
| HED edge detection (color-blob branch only) | CPU (container's OpenCV not built with CUDA). Not on the YOLO scaff path. |

If for any reason Qt mis-renders on your host, restore the round-2 safe mode:

```bash
echo 'JETARM_V5_QT_SAFE=1' >> ~/.jetarm_v5.env
```

### New live tunables in the Vision tab

| Slider | Default | Effect |
|---|---|---|
| `yolo_conf_thresh` | 0.25 | Confidence threshold. 0.5 = stricter (fewer false positives); 0.1 = looser. |
| `yolo_iou_thresh` | 0.7 | NMS IoU. Lower = more overlapping boxes get suppressed. |
| `yolo_max_det` | 100 | Max detections per frame. |
| `inference_max_hz` | 0 | Cap inference rate (0 = uncapped). Lower if GPU thermals get hot. |
| `publish_max_hz` | 0 | Cap result-publisher rate. Match your viewer's actual paint rate to save bandwidth. |
| `publish_scale` | 1.0 | Downsample annotated frame before publish (0.5 → 320×240, ~4× less bytes). |
| `publish_jpeg_quality` | 80 | JPEG quality of `image_result/compressed` (browser/remote viewing). Only encoded while subscribed. |

YOLO knobs (`yolo_conf_thresh` / `yolo_iou_thresh` / `yolo_max_det` /
`inference_max_hz`) apply **instantly while STOPPED** — detection runs
even when sorting is off, so you see the boxes change in the live viewer
as you drag. While RUNNING they queue and land when you press STOP, so a
slider can't change detection mid-pick.

Note: `yolo_imgsz` is **not** tunable — TensorRT engines bake the input
size in at export time. To change it, re-export from Ultralytics with the
new `imgsz` and load the new `.engine` from the Models tab.

### Grip tab (round 15: one-shot pick)

The retry-era knobs (`max_pick_retries`, `vision_confirm_pick`,
`servo_feedback_enabled`, `gripper_full_closed_pulse`, `gripper_slack`,
`gripper_step_pulse`) are gone — the pick is a single
hover → align → descend → close → settle → lift, like the stock
object_sorting app. The grip-reliability knobs are now:

| Slider | Default | Effect |
|---|---|---|
| `gripper_settle` | 0.5 | Dwell after the close command before the lift starts (and after release on place). Raise if objects slip out during lift. Not scaled by `motion_speed`. |
| `grab_depth` | 0.02 | How far below the detected object-z the descend goes, so the jaws wrap the body instead of pinching the top. |

Both support **per-target overrides** for mixed object heights via the
`target_overrides` JSON param, e.g.:

```
target_overrides: '{"scaff": {"grab_depth": 0.025, "gripper_settle": 0.8}, "blue": {"grab_depth": 0.01, "motion_speed": 1.8}}'
```

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
