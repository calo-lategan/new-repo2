# Updating the JetArm to the latest v5

These commands run on the JetArm itself (not the development workstation).

## Standard update

```bash
git -C ~/jetarm_v5_src fetch origin
git -C ~/jetarm_v5_src reset --hard origin/main
bash ~/jetarm_v5_src/custom_sortingv5/install.sh
```

Then relaunch from the desktop icon or:

```bash
~/jetarm_v5/launch_v5.sh
```

`install.sh` is idempotent. It re-runs `colcon build --symlink-install`
for the `app` package, so a fresh shell session will pick up any
launch-file / setup.py changes too.

## Why `reset --hard` instead of `pull`

The `~/jetarm_v5_src` checkout is a **consumer**, not a development
location. It can pick up stray local commits from:

- Earlier `tools/push_logs.sh` runs that committed locally but failed
  to push (network, auth, etc.).
- Aborted CALIBRATE runs that left transient files.

When that happens, plain `git pull` errors out with:

```
hint: You have divergent branches and need to specify how to reconcile them.
fatal: Need to specify how to reconcile divergent branches.
```

`reset --hard origin/main` forces the device to match the GitHub repo
exactly. Any local-only log commits are regenerable - the node writes
fresh session logs at every launch.

If you want to be extra safe, list local-only commits before the reset:

```bash
git -C ~/jetarm_v5_src log --oneline origin/main..HEAD
```

If that list shows only `logs: session ...` commits, the reset is safe.

## Verifying the launch came up

After `launch_v5.sh` settles, every node + camera should be on
`ROS_DOMAIN_ID=0`:

```bash
ros2 node list | grep -E "custom_sortingv5|calibration|lab_manager"
ros2 topic hz /depth_cam/rgb/image_raw     # ~30 Hz
ros2 topic hz /depth_cam/depth/image_raw   # >0 Hz
ros2 service list | grep lab_manager       # 7 services
```

The three calibration tabs in the tuner UI:

- **Position** - drives the vendor `/calibration/*` services.
  CALIBRATE POSITION writes `transform.yaml`.
- **Color** - drives the vendor `/lab_manager/*` services.
  CALIBRATE COLOR writes `lab_config.yaml`.
- **Depth** - drives v5's `~/depth_plane_refit` (which uses vendor
  `SearchPlane`). CALIBRATE DEPTH (plane refit) writes the `plane:`
  key into `transform.yaml`.

Each tab's CALIBRATE button only touches its own YAML.

Each calibration tab also shows a **live preview** (the same image the
Hiwonder web tool streamed): the Position tab shows the AprilTag
detection + projected workspace rectangle (`/calibration/image_result`),
and the Color tab shows the live LAB mask (`/lab_manager/image_result`,
after you press ENTER). If a preview is blank, install Pillow's Tk
bindings: `sudo apt install -y python3-pil.imagetk`.

## Calibration in a separate window

Two ways to pop the calibration tabs into their own window on the same
ROS domain (so you can calibrate while watching the main app):

1. In the app: each calibration tab has an **"Open in separate window"**
   button.
2. From a terminal:
   ```bash
   bash ~/jetarm_v5_src/tools/calibration_tools.sh           # Position
   bash ~/jetarm_v5_src/tools/calibration_tools.sh color     # Color
   bash ~/jetarm_v5_src/tools/calibration_tools.sh depth     # Depth
   ```
   It builds only the Position/Color/Depth tabs and acts as a service
   client of the running nodes - no second camera, no second sorting
   node. Domain defaults to 0 (override with `ROS_DOMAIN_ID=`).

## Pushing logs

The **PUSH LOGS** button (and `tools/push_logs.sh`) publishes the most
recent session logs to a DEDICATED `jetarm-logs` branch on GitHub - NOT
`main` - so it never conflicts with a moving main and never pollutes the
main history. View them at the repo's `jetarm-logs` branch under
`logs/`.

If the button reports `git push failed`, the device clone almost
certainly has no push credentials (it can pull over HTTPS but can't
push). One-time fix, either:

- Configure a credential helper / Personal Access Token:
  ```bash
  git -C ~/jetarm_v5_src config credential.helper store
  # next push will prompt once for username + PAT, then remembers it
  ```
- Or switch the remote to SSH (if the device has a key on GitHub):
  ```bash
  git -C ~/jetarm_v5_src remote set-url origin git@github.com:calo-lategan/new-repo2.git
  ```

The working-tree copy under `~/jetarm_v5_src/logs/` survives until the
next `reset --hard`, so even a failed push leaves the files retrievable
on the device.

## Troubleshooting

### "depth topic NOT publishing"
The Orbbec depth driver didn't come up. Usually a USB enumeration
issue:

```bash
sudo systemctl restart start_app_node.service
# wait 25-30s, then:
ros2 topic hz /depth_cam/depth/image_raw
```

If still missing, unplug + replug the Astra USB cable, then relaunch.

### "vendor calibration_node service not reachable"
The calibration node didn't come up. Check:

```bash
ros2 node list | grep calibration
ros2 service list | grep calibration
```

If absent, the v5 launch was started before the workspace was sourced.
Re-run `~/jetarm_v5/launch_v5.sh` from a fresh terminal.

### "lab_manager service not reachable"
Same as above for `/lab_manager`:

```bash
ros2 node list | grep lab_manager
```

If absent, restart the launcher.

### Multiple `ROS_DOMAIN_ID`s
All v5 components are pinned to domain 0 by `launch_v5.sh`. If you've
manually run `ros2 ...` commands in a separate terminal, they may have
inherited a different domain from your shell rc. Match by exporting:

```bash
export ROS_DOMAIN_ID=0
```

before any manual `ros2 topic` / `ros2 service` call.

### "Plane refit failed (no depth?)"
The depth topic is OFF, the depth camera info topic isn't published,
or `SearchPlane` couldn't import. The Depth-tab status panel breaks
this down: check `depth status`, `depth topic`, `depth->color TF`.
Most often: depth driver isn't up - see "depth topic NOT publishing"
above.

### PUSH LOGS button does nothing
If the script can't find the repo, the bottom-bar status will show
`PUSH LOGS: no repo found`. Either the checkout was deleted, or you
cloned to a non-standard path. Set:

```bash
export JETARM_V5_REPO=/path/to/your/clone
```

before launching v5, or run the script manually:

```bash
bash ~/jetarm_v5_src/tools/push_logs.sh
```
