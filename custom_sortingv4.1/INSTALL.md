# Custom Sorting v4.1 — install on the Hiwonder JetArm (Jetson Orin Nano)

v4.1 is the stable cleanup of v4. Folder layout uses **`v4_1` for Python module names** (Python rejects a literal dot as a package separator) and **`v4.1` for everything else** — launcher script, launch file, desktop shortcut, the folder itself.

| File | What it is |
|---|---|
| `custom_sortingv4_1.py` | The sorting node. Cameras always-on, gated inference, hot-swap engine. |
| `tune_uiv4_1.py` | Tkinter tuner UI. Buttons to open rqt_image_view / image_view / browser. |
| `custom_sorting_nodev4.1.launch.py` | The ROS 2 launch file. |
| `launch_v4.1.sh` | One-click bash launcher. Stops + disables the factory service first. |
| `image_view_chain.sh` | rqt → image_view → browser chain, IP autodetected. |
| `re-enable-factory.sh` | Opt-in: put Hiwonder's factory app back. |
| `install.sh` | One-paste idempotent installer. |
| `profiles/` (created at install) | `~/jetarm_v4_profiles/{default,fast,precision}.yaml` |

---

## One-paste install

Open a terminal **inside the Hiwonder container** and run:

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/calo-lategan/new-repo2/main/custom_sortingv4.1/install.sh) --sudoers
```

This will:

1. Clone the repo into `~/jetarm_v4_1_src`
2. Symlink `custom_sortingv4_1.py` and `tune_uiv4_1.py` into `~/ros2_ws/src/app/app/`
3. Symlink `custom_sorting_nodev4.1.launch.py` into `~/ros2_ws/src/app/launch/`
4. Idempotently add the two `console_scripts` entries (`custom_sortingv4_1`, `tune_uiv4_1`) to `setup.py`
5. Seed `~/jetarm_v4_profiles/` with the default + fast + precision profiles
6. Install `~/jetarm_v4_1/launch_v4.1.sh` + `image_view_chain.sh` + `re-enable-factory.sh`
7. Drop a **JetArm Sort v4.1** desktop shortcut on your Desktop and in the app menu
8. Run `colcon build --packages-select app --symlink-install`
9. (because of `--sudoers`) Install NOPASSWD rules so the launcher can `stop`/`disable`/`restart`/`start` `start_app_node.service` without prompting

Once it finishes, double-click **JetArm Sort v4.1** on your desktop.

---

## What launching does (in order)

1. **Stops + disables** `start_app_node.service` (idempotent — runs every launch). The Hiwonder factory app grabs the camera + serial port + runs its own pick/sort code, which conflicts with v4.1. We never want it running alongside us.
2. Waits for the Hiwonder container to be ready (`/opt/ros/humble/setup.bash` exists, `~/ros2_ws/` exists).
3. Sources ROS 2 humble + the local install tree.
4. Boots: the v4.1 sorting node, the tuner UI, **and `web_video_server`** (because the factory service we just disabled was the thing that used to start it — we now start it ourselves on port 8080).
5. After ~8 s the `image_view_chain.sh` fires automatically and opens `rqt_image_view` in its own terminator window (you can disable this with `image_view:=false`).

---

## Camera view (no IP lookup, no fights with Docker)

The tuner UI has a `Camera view` row with four big buttons. **A viewer auto-opens at launch**; these buttons replace/swap it:

- **Open rqt_image_view** (green) — Qt window with topic dropdown. Runs inside `terminator` with `QT_X11_NO_MITSHM=1`, `QT_QPA_PLATFORM=xcb`, and `LIBGL_ALWAYS_SOFTWARE=1` exported — that's the combination that makes rqt actually render inside the Hiwonder Docker container's forwarded X11. Output is also tee'd to `/tmp/jetarm_v4_1_rqt.log` so you can `cat` it if the window misbehaves.
- **Open image_view** (blue) — the C++ classic, same X11 env, output tee'd to `/tmp/jetarm_v4_1_image_view.log`.
- **Open browser** (purple) — opens `http://<jetson-ip>:8080/stream?topic=/custom_sortingv4_1/image_result&th=100`. IP detected via `hostname -I`. `web_video_server` is now started by our launch on port 8080 (it used to come from the factory service we disable), so this works standalone.
- **Close viewer** (red) — kills the currently tracked viewer subprocess.

The publisher uses default RELIABLE QoS with depth=10 — that matches what `image_view`, `rqt_image_view`, and `web_video_server` request by default in humble. (The earlier round briefly tried `sensor_data` (BEST_EFFORT) which turned out to be incompatible — those consumers demand RELIABLE.)

### Disable auto-popup on launch

```bash
~/jetarm_v4_1/launch_v4.1.sh image_view:=false
```

(The buttons still work.)

### Point the viewer at a different topic

```bash
~/jetarm_v4_1/launch_v4.1.sh image_view_topic:=/depth_cam/rgb/image_raw
```

(Or change the topic field in the tuner UI before clicking Open.)

---

## Debugging from a second terminal — match the launcher's ROS env

**The single most common reason for "blank viewer" / "topic not published yet" is `ROS_DOMAIN_ID` mismatch.** The launcher prints its domain in the banner:

```
[env] ROS_DOMAIN_ID=0    <-- match this in other terminals!
```

To probe the running stack from a second terminal, source the helper that ships with v4.1:

```bash
source ~/jetarm_v4_1/match_launcher_env.sh
```

That sets `ROS_DOMAIN_ID` to whatever the launcher used, sources ROS humble + the workspace + orbbec_ws, and prints the visible nodes so you can confirm discovery worked.

Override the domain by either:

- One-shot: `ROS_DOMAIN_ID=5 ~/jetarm_v4_1/launch_v4.1.sh` (then set the same `ROS_DOMAIN_ID=5` before sourcing `match_launcher_env.sh`)
- Persistent: put `ROS_DOMAIN_ID=5` in `~/.jetarm_v4_1.env` — both the launcher and `match_launcher_env.sh` source it.

## Debugging frame flow

If a viewer still shows a blank window AFTER matching the domain, the live frames may not be flowing. Diagnose:

```bash
# 1. Is the topic actually publishing?
ros2 topic hz /custom_sortingv4_1/image_result          # expect ~15-30 Hz

# 2. What does the first message look like?
ros2 topic echo --once /custom_sortingv4_1/image_result --no-arr | head -20
#    encoding: bgr8 / height: 480 / width: 640 / step: 1920

# 3. Direct rqt_image_view test with X11 env + explicit QoS
QT_X11_NO_MITSHM=1 rqt_image_view /custom_sortingv4_1/image_result

# 4. web_video_server in browser
firefox "http://$(hostname -I | awk '{print $1}'):8080/stream?topic=/custom_sortingv4_1/image_result&th=100"

# 5. Confirm factory service is stopped + disabled
systemctl is-active   start_app_node.service           # expect "inactive"
systemctl is-enabled  start_app_node.service           # expect "disabled"

# 6. Confirm web_video_server is listening (we start it now)
ss -tlnp | grep 8080
curl -sI http://localhost:8080/ | head -1              # expect "HTTP/1.1 200"

# 7. Confirm RELIABLE QoS on the publisher (matches image_view / web_video_server)
ros2 topic info /custom_sortingv4_1/image_result --verbose
#    Look for: Reliability: RELIABLE, History (Depth): 10

# 8. If rqt window is blank/black, dump its log
cat /tmp/jetarm_v4_1_rqt.log
```

On the node side, the first time `_publish_image` succeeds it prints:

```
[v4_1][publish] first frame published: shape=(480, 640, 3) step=1920 contig=True topic=/custom_sortingv4_1/image_result
```

If you see that line, the publish path is healthy and any remaining blank window is on the viewer side (re-check X11 env / browser URL throttle).

For verbose stage logs:

```bash
JETARM_V4_DEBUG=1 ~/jetarm_v4_1/launch_v4.1.sh
```

---

## Bringing back the factory app

The launcher disables `start_app_node.service` on every run. To go back to Hiwonder's factory bringup:

```bash
~/jetarm_v4_1/re-enable-factory.sh
```

This `unmask`s, `enable`s, and `start`s the service. Note that running `launch_v4.1.sh` again will re-disable it — that's intentional. To keep the factory app permanently, don't launch v4.1.

---

## Updating later (no rebuild)

The installer symlinks the sources and builds with `--symlink-install`, so pulling new code is enough:

```bash
git -C ~/jetarm_v4_1_src pull
~/jetarm_v4_1/launch_v4.1.sh
```

Only re-run `install.sh` if `setup.py` / `package.xml` / launch arguments changed upstream.

---

## Uninstall

```bash
rm -rf ~/jetarm_v4_1 ~/jetarm_v4_profiles ~/jetarm_v4_1_src \
       ~/ros2_ws/src/app/app/custom_sortingv4_1.py \
       ~/ros2_ws/src/app/app/tune_uiv4_1.py \
       ~/ros2_ws/src/app/launch/custom_sorting_nodev4.1.launch.py \
       ~/Desktop/jetarm-sort-v4.1.desktop \
       ~/.local/share/applications/jetarm-sort-v4.1.desktop
sudo rm -f /etc/sudoers.d/jetarm-v4.1
# remove the two console_scripts lines from ~/ros2_ws/src/app/setup.py
cd ~/ros2_ws && colcon build --packages-select app
~/jetarm_v4_1/re-enable-factory.sh   # if you want the Hiwonder factory app back
```
