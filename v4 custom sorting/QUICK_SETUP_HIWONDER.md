# JetArm v4 - Quick Setup for Hiwonder Container

This is the **container-specific** install guide. Follow it exactly and you cannot break the Hiwonder image. INSTALL.md is the longer reference; this is the short, opinionated version.

## What you should already know about your robot

The Hiwonder image is layered like this. Don't fight it — match it.

```
host OS  (Ubuntu on the Jetson Orin Nano)
└── container  (the "hiwonder" lxc/docker the desktop terminal drops you into)
    ├── /opt/ros/humble/          ROS2 install
    ├── /home/ubuntu/ros2_ws/     workspace (default zsh)
    │   ├── src/app/app/          ← v4 .py files live here
    │   ├── src/app/launch/       ← v4 launch file lives here
    │   ├── install/setup.zsh     ← what you source after `colcon build`
    │   └── ...
    ├── start_app_node.service    systemd unit that runs bringup.launch.py
    └── bringup.launch.py         brings up SDK + start_app + (TimerAction
                                  18s) depth_camera_launch
```

### The camera ↔ service contract (CRITICAL)

- The camera is **only** published when `start_app_node.service` is up. No service = no `/depth_cam/rgb/image_raw`. Don't try to launch the depth_cam node separately, it will fight bringup.
- Bringup has a built-in `TimerAction(period=18.0)` that delays the camera node so the SDK / kinematics nodes come up first. So even after the service is "active", the camera takes ≥18 s to actually publish.
- If anything else has grabbed the camera (e.g. you ran a non-bringup launch that includes its own `depth_camera.launch.py`), `sudo systemctl restart start_app_node.service` is the canonical reclaim. The new bringup will kill the old depth_cam and start its own.
- `launch_v4.sh` does all of this for you: restart → wait for service active → wait for camera topic to actually publish → then launch v4.

## Install steps (copy-paste, in order)

Open a terminal **inside** the Hiwonder container (the desktop one is fine).

```zsh
# 1. Sanity: confirm you're in the container, not the host
[[ -d /home/ubuntu/ros2_ws ]] && echo "OK - in container" || echo "WRONG - this is the host"

# 2. Copy v4 sources into the app package (NOT the system /opt path)
cd ~
git clone https://github.com/calo-lategan/new-repo2.git  # or git pull if cloned
cd new-repo2/"v4 custom sorting"
cp custom_sortingv4.py               ~/ros2_ws/src/app/app/custom_sortingv4.py
cp tune_uiv4.py                      ~/ros2_ws/src/app/app/tune_uiv4.py
cp custom_sorting_nodev4.launch.py   ~/ros2_ws/src/app/launch/custom_sorting_nodev4.launch.py

# 3. Add the two console_scripts to setup.py (do this once; idempotent edit)
#    See setup.py.snippet for the exact lines. Then:
nano ~/ros2_ws/src/app/setup.py
# add inside entry_points -> console_scripts:
#   'custom_sortingv4 = app.custom_sortingv4:main',
#   'tune_uiv4        = app.tune_uiv4:main',

# 4. Seed your profiles dir
mkdir -p ~/jetarm_v4_profiles
cp profiles/default.yaml   ~/jetarm_v4_profiles/default.yaml
cp profiles/fast.yaml      ~/jetarm_v4_profiles/fast.yaml
cp profiles/precision.yaml ~/jetarm_v4_profiles/precision.yaml

# 5. Build the app package only - don't `--symlink-install` and don't rebuild
#    the whole workspace, you don't need to
cd ~/ros2_ws
colcon build --packages-select app
source install/setup.zsh

# 6. Install the launcher + desktop shortcut
mkdir -p ~/jetarm_v4
cp ~/new-repo2/"v4 custom sorting"/launch_v4.sh ~/jetarm_v4/launch_v4.sh
chmod +x ~/jetarm_v4/launch_v4.sh
cp ~/new-repo2/"v4 custom sorting"/jetarm-sort-v4.desktop ~/Desktop/jetarm-sort-v4.desktop
chmod +x ~/Desktop/jetarm-sort-v4.desktop

# 7. (One-time, optional) make the desktop shortcut truly one-click:
sudo visudo -f /etc/sudoers.d/jetarm-v4
# add this line, replacing `ubuntu` with your username:
ubuntu ALL=(ALL) NOPASSWD: /bin/systemctl restart start_app_node.service
```

That's it. Double-click the desktop shortcut.

## What the desktop shortcut actually does (so you can debug it)

```
[container] wait up to 30s for /opt/ros/humble/setup.* and ~/ros2_ws to exist
[env]       cd ~/ros2_ws
[env]       source /opt/ros/humble/setup.bash   # the launcher is a bash script
[env]       source ~/ros2_ws/install/setup.bash # so it must source the .bash variants
                                                 #   (USE_ZSH=1 re-execs under zsh
                                                 #    if you really want setup.zsh)
[env]       export CAMERA_TYPE=GEMINI CHASSIS_TYPE=Slide_Rails need_compile=False
[service]   sudo systemctl restart start_app_node.service
[service]   wait up to 30s for systemctl is-active
[controllers] wait up to 25s for /ros_robot_controller* topics
[camera]    wait up to 45s for /depth_cam/rgb/image_raw to publish frames
[launcher]  ros2 launch app custom_sorting_nodev4.launch.py
```

Each stage prints `[stage] message` to the terminal. If something hangs, the last `[stage]` line tells you exactly where it stopped.

## Debug mode

When weird things happen, run with verbose debug logging:

```zsh
# from the shortcut:
JETARM_V4_DEBUG=1 ~/jetarm_v4/launch_v4.sh

# or pass to the launch:
ros2 launch app custom_sorting_nodev4.launch.py debug:=true
```

You'll get `[v4][stage] message` lines at every internal pivot:

| Tag | When |
|---|---|
| `[v4][init]` | Constructor steps: HED model load, kinematics service wait, inference worker spawn |
| `[v4][engine-load]` | YOLO `.engine` load timing + warmup |
| `[v4][engine-swap]` | Queueing / completing a hot-swap |
| `[v4][camera]` | First frame received, first camera_info received, reshape failures |
| `[v4][roi]` | `transform.yaml` load and ROI box computation |
| `[v4][enter]` | `/custom_sortingv4/enter` service was hit |
| `[v4][inference]` | First inference complete; periodic stats in debug mode |
| `[v4][transport]` | Per-pick lifecycle (BEGIN, pick OK/FAIL, place OK/FAIL) |
| `[v4][heartbeat]` | Every 5 s: camera FPS, frames received, ROI status, current target |

If a thread crashes you'll see `[v4][sorting-loop] CRASHED` or `[v4][transport] CRASHED` followed by the exception traceback, instead of the thread dying silently like rclpy normally does.

## Common failure modes

### "kinematics/set_pose_target NEVER appeared"
The service didn't bring the SDK up. Run:
```zsh
sudo journalctl -u start_app_node.service -n 80 --no-pager
```
Common cause: stale process holding the controller serial. Reboot the container.

### "FIRST FRAME" never appears
The camera node isn't publishing. The launcher should have caught this in step 7. Check:
```zsh
ros2 topic hz /depth_cam/rgb/image_raw
```
If empty, the service didn't reclaim the camera. Try `SKIP_SERVICE_RESTART=0 ~/jetarm_v4/launch_v4.sh` to force the restart again.

### "transform.yaml NOT FOUND"
You haven't run the upstream calibration step yet:
```zsh
ros2 launch app calibration_node.launch.py
```
Do that once before using v4 - or any sorting app, this isn't v4-specific.

### "CAMERA_TYPE env var is missing"
The launcher couldn't export it. Make sure you're using `launch_v4.sh`, not running the node directly.

## Folder layout you'll end up with

```
~/ros2_ws/
├── install/...                  built by `colcon build`
└── src/app/
    ├── setup.py                  (you added 2 entry_points)
    ├── app/
    │   ├── custom_sortingv4.py   ← v4 node
    │   ├── tune_uiv4.py          ← v4 tuner UI
    │   ├── custom_sortingv2.py   (v2, untouched)
    │   ├── tune_ui.py            (v2, untouched)
    │   └── ...
    └── launch/
        └── custom_sorting_nodev4.launch.py

~/jetarm_v4_profiles/   ← profile YAMLs (auto-loaded; UI manages them)
~/jetarm_v4/            ← launch_v4.sh
~/Desktop/jetarm-sort-v4.desktop
```

## Do NOT do these things

- Don't edit `/opt/ros/humble/share/app/...` directly. That's the installed copy; rebuild via `colcon build`.
- Don't `pip install` ultralytics into the system python. The container's python already has it tuned for the Jetson; you'll break TensorRT bindings.
- Don't run `sudo apt upgrade ros-*`. The Hiwonder image pins versions; an upgrade can break the depth_cam driver.
- Don't try to run the depth camera standalone. Always go through `start_app_node.service`.
- Don't disable the service to "speed things up". The camera depends on it.
