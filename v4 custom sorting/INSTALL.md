# Custom Sorting v4 - install on the Jetson Orin Nano

> **If you are using the Hiwonder container image (almost everyone is),
> read `QUICK_SETUP_HIWONDER.md` first.** It explains the camera ↔
> `start_app_node.service` contract, container readiness, and the
> exact folder layout. This file is the long-form reference.

v4 is a from-research-up rewrite. v2 still works — install v4 alongside it without touching the v2 files.

---

## One-paste install (recommended)

Open a terminal **inside the Hiwonder container** and run:

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/calo-lategan/new-repo2/main/v4%20custom%20sorting/install.sh) --sudoers
```

That's it. It will:

1. Clone the repo into `~/jetarm_v4_src`
2. Copy `custom_sortingv4.py` + `tune_uiv4.py` to `~/ros2_ws/src/app/app/`
3. Copy the launch file to `~/ros2_ws/src/app/launch/`
4. Idempotently add the two `console_scripts` entries to `setup.py`
5. Seed `~/jetarm_v4_profiles/` with `default.yaml`, `fast.yaml`, `precision.yaml`
6. Install `~/jetarm_v4/launch_v4.sh`
7. Drop the `JetArm Sort v4` desktop shortcut on your Desktop and in the app menu
8. Run `colcon build --packages-select app`
9. (because of `--sudoers`) Install a `NOPASSWD` sudoers rule for the one `systemctl restart start_app_node.service` command so the desktop shortcut never prompts for a password

Drop `--sudoers` if you want to skip the sudoers step and enter your password each launch.

Once it finishes:

- Double-click **JetArm Sort v4** on your desktop, OR
- Run `~/jetarm_v4/launch_v4.sh` from a terminal

If you want to see every internal step printed live to the terminal as the robot starts up (recommended for the first few runs), launch with debug:

```bash
JETARM_V4_DEBUG=1 ~/jetarm_v4/launch_v4.sh
```

### Updating later (no rebuild)

The installer symlinks the v4 sources into `~/ros2_ws/src/app/...` and builds the workspace with `--symlink-install`. That means pure-Python edits don't need a rebuild — pulling fresh code is enough:

```bash
git -C ~/jetarm_v4_src pull
~/jetarm_v4/launch_v4.sh    # next launch picks up the new code
```

Re-run the one-paste installer **only when** something structural changed upstream — new `setup.py` entry point, new launch file, new dependency. Re-running is also always safe (idempotent): existing symlinks and profile YAMLs are preserved.

### Uninstalling

```bash
rm -rf ~/jetarm_v4 ~/jetarm_v4_profiles ~/jetarm_v4_src \
       ~/ros2_ws/src/app/app/custom_sortingv4.py \
       ~/ros2_ws/src/app/app/tune_uiv4.py \
       ~/ros2_ws/src/app/launch/custom_sorting_nodev4.launch.py \
       ~/Desktop/jetarm-sort-v4.desktop \
       ~/.local/share/applications/jetarm-sort-v4.desktop
sudo rm -f /etc/sudoers.d/jetarm-v4
# also remove the two console_scripts lines from ~/ros2_ws/src/app/setup.py
# and re-run: cd ~/ros2_ws && colcon build --packages-select app
```

---

| File | Purpose |
|------|---------|
| `custom_sortingv4.py` | Faster + hot-swappable sorting node (see RESEARCH.md for the design rationale). |
| `tune_uiv4.py` | Tkinter UI with **START / STOP / CALIBRATE / SAVE-AS-DEFAULT**, model swap, profile manager. |
| `custom_sorting_nodev4.launch.py` | Launch file - SDK + camera + node + UI in one go. Accepts `profile:=fast` etc. |
| `launch_v4.sh` | One-click bash wrapper. Restarts `start_app_node.service`, waits for the controller topics, then launches. |
| `jetarm-sort-v4.desktop` | Desktop shortcut for the script above. |
| `profiles/default.yaml` | Boot profile - the node auto-loads `~/jetarm_v4_profiles/default.yaml`. |
| `profiles/fast.yaml` | Sample fast profile (load with `profile:=fast`). |
| `profiles/precision.yaml` | Sample precision profile. |
| `setup.py.snippet` | Lines to paste into the `app` package's `setup.py`. |
| `RESEARCH.md` | Annotated research that drove the design. |

## Highlights vs v2

- **Hot-swap AI models with no restart**: pick a different `.engine` from the Models tab; the inference worker rebuilds the model between frames, runs a warmup pass, and frees the old engine's CUDA memory. Train a new YOLO, export to `.engine`, drop it in `/home/ubuntu/third_party_ros2/data/`, click — that's the iteration loop.
- **Profiles + persistent settings**: every tunable lives in YAML profiles under `~/jetarm_v4_profiles/`. **SAVE AS DEFAULT** writes the current state to `default.yaml`, which is auto-loaded next boot. **SAVE PROFILE** lets you stash named profiles (`my-blocks`, `scaff-only`, ...). Launch with `profile:=name` to boot into one.
- **Faster per-frame**: zero-copy image conversion (`np.frombuffer` view, no `cv_bridge`), proper QoS (`qos_profile_sensor_data`, depth=1), explicit callback groups so a UI param-set never stalls the camera, dedicated inference thread with warmup, no per-frame INFO logs.
- **Per-target overrides** (e.g. slower for scaff, faster for blocks): set `target_overrides` to a JSON like `{"scaff": {"motion_speed": 0.9}, "blue": {"motion_speed": 1.8}}`.
- **All v2 features kept**: vision + bus-servo grip feedback with retries, self-calibration of bin offsets, multi-frame averaging.

## Install on the robot

1. Copy the files into the `app` ROS2 package on the Jetson:

   ```bash
   cp custom_sortingv4.py             ~/ros2_ws/src/app/app/custom_sortingv4.py
   cp tune_uiv4.py                    ~/ros2_ws/src/app/app/tune_uiv4.py
   cp custom_sorting_nodev4.launch.py ~/ros2_ws/src/app/launch/custom_sorting_nodev4.launch.py
   ```

2. Add the entry points to `~/ros2_ws/src/app/setup.py` inside `entry_points -> console_scripts`:

   ```python
   'custom_sortingv4 = app.custom_sortingv4:main',
   'tune_uiv4        = app.tune_uiv4:main',
   ```

3. Seed the profiles directory (one-time):

   ```bash
   mkdir -p ~/jetarm_v4_profiles
   cp profiles/default.yaml   ~/jetarm_v4_profiles/default.yaml
   cp profiles/fast.yaml      ~/jetarm_v4_profiles/fast.yaml
   cp profiles/precision.yaml ~/jetarm_v4_profiles/precision.yaml
   ```

4. Build and source:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select app
   source install/setup.bash
   ```

5. Launch:

   ```bash
   ros2 launch app custom_sorting_nodev4.launch.py
   # boot into a named profile:
   ros2 launch app custom_sorting_nodev4.launch.py profile:=fast
   # boot already-running (skip the START button):
   ros2 launch app custom_sorting_nodev4.launch.py start:=true
   # headless (no UI, e.g. SSH without X):
   ros2 launch app custom_sorting_nodev4.launch.py tune_ui:=false
   ```

## One-click desktop launcher

```bash
mkdir -p ~/jetarm_v4
cp launch_v4.sh ~/jetarm_v4/launch_v4.sh
chmod +x ~/jetarm_v4/launch_v4.sh

cp jetarm-sort-v4.desktop ~/Desktop/jetarm-sort-v4.desktop
chmod +x ~/Desktop/jetarm-sort-v4.desktop
mkdir -p ~/.local/share/applications
cp jetarm-sort-v4.desktop ~/.local/share/applications/
```

On Ubuntu 24.04 (GNOME), right-click the icon on the desktop -> *Allow Launching* the first time.

### What the launcher does

1. `cd ~/ros2_ws`
2. `source /opt/ros/humble/setup.bash` and `source install/setup.bash`
3. `sudo systemctl restart start_app_node.service`
4. **Waits** for systemd to report active and for `ros_robot_controller` topics to appear (the equivalent of waiting for the beep)
5. `ros2 launch app custom_sorting_nodev4.launch.py "$@"`

For password-free one-click, add this sudoers line (replace `ubuntu` with your username):

```bash
sudo visudo -f /etc/sudoers.d/jetarm-v4
# add:
ubuntu ALL=(ALL) NOPASSWD: /bin/systemctl restart start_app_node.service
```

## Operating it (the safe loop)

1. Click the desktop shortcut. The node boots **STOPPED**, with the default profile applied; the tuner UI opens.
2. (Optional) Press **CALIBRATE** to nudge the bin offsets based on the current scene.
3. Adjust any sliders, swap engines on the **Models** tab, load/save profiles on the **Profiles** tab.
4. Press **START SORTING** when ready.
5. Press **STOP** any time to pause; tweak settings or recalibrate; **START** to resume.
6. Once you find a good combo, press **SAVE AS DEFAULT** so it loads on next boot.

## Hot-swapping AI models (the iteration loop)

You said you wanted easy iteration once you've trained a new YOLO and exported to `.engine`:

1. Drop the new engine into your engines dir (default `/home/ubuntu/third_party_ros2/data/`).
2. Open the **Models** tab in the tuner UI.
3. Click **Refresh** -> select the new file -> **Load selected**.
   - Or paste a full path into "Manual path" and click **Load entered path**.
4. The inference worker swaps it between frames, runs a warmup pass, releases the old engine's GPU memory. No node restart, no service restart.

To bake the new engine into the boot profile, click **SAVE AS DEFAULT** afterward.

You can also trigger from the CLI:

```bash
ros2 service call /custom_sortingv4/load_engine \
   interfaces/srv/SetStringBool \
   "{data_str: '/path/to/new.engine', data_bool: true}"
```

## Profile management

```bash
# List saved profiles
ls ~/jetarm_v4_profiles

# Save current settings as a named profile
ros2 service call /custom_sortingv4/save_profile \
   interfaces/srv/SetStringBool \
   "{data_str: 'my-fast', data_bool: true}"

# Load a profile at runtime
ros2 service call /custom_sortingv4/load_profile \
   interfaces/srv/SetStringBool \
   "{data_str: 'my-fast', data_bool: true}"

# Save current as boot default
ros2 service call /custom_sortingv4/save_as_default std_srvs/srv/Trigger
```

## Live camera view (auto-opens, with browser fallback)

v4 auto-opens a desktop image-viewer window on the annotated frame about 6 seconds after launch. You don't have to do anything — and you don't need to know the Jetson's IP.

Under the hood, the launch invokes `~/jetarm_v4/image_view_chain.sh` which runs this fallback chain:

1. `rqt_image_view <topic>` — opens a Qt window with a topic dropdown.
2. If that's not installed: `ros2 run image_view image_view` — the C++ classic.
3. **When the GUI viewer exits (you close it, or it crashes), the chain auto-opens the system browser** at `http://<jetson-ip>:8080/stream?topic=/custom_sortingv4/image_result`. The Jetson IP is detected via `hostname -I` — no manual lookup, no typing URLs.
4. If neither native viewer is installed at all, it skips straight to the browser.

So worst case, you always end up with a visible feed in something.

The previous cv2 popup was dropped because its Qt event loop couldn't be pumped from a background thread on the Jetson's containerized desktop. The native viewers run in their own process, so they can't be starved by the sorting loop.

### Tuning

Disable the auto-opened viewer entirely (e.g. SSH/headless):

```bash
~/jetarm_v4/launch_v4.sh image_view:=false
```

Show a different topic from the start (e.g. raw camera, or a depth topic):

```bash
~/jetarm_v4/launch_v4.sh image_view_topic:=/depth_cam/rgb/image_raw
```

Pick a specific browser for the fallback (otherwise it tries `xdg-open` → `sensible-browser` → `firefox` → `chromium`):

```bash
IMAGE_VIEW_BROWSER=firefox ~/jetarm_v4/launch_v4.sh
```

Disable just the browser fallback (keep the GUI viewer):

```bash
IMAGE_VIEW_BROWSER_DISABLE=1 ~/jetarm_v4/launch_v4.sh
```

The `display:=true` launch argument is preserved but is now a no-op.

## Camera is "always on" — sorting is what you toggle

The v4 node subscribes to `/depth_cam/rgb/image_raw` the moment it starts, **not** when you press START. So:

- The live viewer (`rqt_image_view` / browser) shows the camera feed as soon as the launcher reaches the node — even with sorting stopped.
- A 15 Hz republisher pushes the latest raw frame to `~/image_result` whenever sorting is off, so the viewer always has something to display.
- YOLO inference is gated behind `enable_sorting` — when stopped, no frames go to the model and the GPU is idle.
- Press **START SORTING** in the tuner → image_callback starts submitting frames to YOLO → annotated frames replace the raw feed in the viewer → picks happen.
- Press **STOP** → no more inference, no more picks, but the camera feed stays visible.

### Service interaction

The launcher uses `systemctl start` (not `restart`) so an already-active `start_app_node.service` is never disturbed. The duplicate `depth_camera_launch` that v4 used to include was removed — Hiwonder's bringup chain is the sole owner of the camera node, and v4 just subscribes to its topic. No more `Could not find requested resource in ament index` errors from a duplicate `/depth_cam/camera_container`.

If the service is in a bad state and you really do need a kill-and-restart:

```bash
FORCE_SERVICE_RESTART=1 ~/jetarm_v4/launch_v4.sh
```

## Debugging

Every critical stage of v4 prints a tagged line to stderr in `[v4][stage] message` format. To see them, just watch the terminal that the launcher opened — they're always-on. To get the *verbose* stream (per-frame timing, periodic stats, every kinematics IO), run with debug mode:

```bash
# from the launcher:
JETARM_V4_DEBUG=1 ~/jetarm_v4/launch_v4.sh

# at launch time:
ros2 launch app custom_sorting_nodev4.launch.py debug:=true

# at runtime (no restart):
ros2 param set /custom_sortingv4 debug true
```

A heartbeat line prints every 5 s with a one-glance health summary:

```
[v4][heartbeat] enter=True sorting=True cam_fps=29.7 frames=4488
                roi=ok intrinsic=ok inference_age_ms=42
                target=scaff transport=False
```

If a thread crashes (sorting loop, transport, inference worker) you'll see a `CRASHED` stage line with the full exception traceback instead of the silent thread death that rclpy normally gives you.

## Notes / gotchas

- The node defaults to the YOLO engine at `/home/ubuntu/third_party_ros2/data/best_scaff2.engine`. Override with `engine_path:=...` at launch, or hot-swap from the UI.
- If the `ros_robot_controller/bus_servo/get_state` service isn't up within 5s the node logs a warning and falls back to vision-only retries.
- Per-target overrides (`target_overrides`) are JSON in a string param. Edit it via `ros2 param set` or in your profile YAML. Example:
  ```yaml
  target_overrides: '{"scaff": {"motion_speed": 0.9, "max_pick_retries": 4}, "blue": {"motion_speed": 1.8}}'
  ```
- On servo overheating (>65 C reported by `GetBusServoState`) the gripper logic treats the close as "stalled" and backs off.
- See `RESEARCH.md` for the actual ROS2 / rclpy / Ultralytics references that drove every design choice.
