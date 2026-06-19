# JetArm Custom Sorting — v5

The current (and only) version of the custom YOLO sorting stack for the
Hiwonder JetArm (Jetson Orin Nano). v5 replaces v2 / v4 / v4.1.

## What v5 is

- **One YOLO model for all detection** — no OpenCV/LAB color path. Class
  names come from `model.names`; one model can carry cubes + scaff + more.
- **Buffered Model tab** — engine, YOLO knobs and enabled classes are
  edited locally and applied only on **SAVE** (engine hot-swaps between
  frames, no app restart). Saved to `~/jetarm_v5_profiles/yolo.yaml`.
- **Per-class targets** — set each class's drop point (x, y, z) and grip
  strength in the Places tab.
- **Standard grasp by default**, with an opt-in **force-limited "close
  until contact" BETA** (per-class max strength + servo over-temp
  protection). The hardware exposes no servo load, so this is an honest
  contact-stop, not a force sensor.
- **AprilTag calibrate** — the CALIBRATE button runs the vendor
  calibration node (writes `transform.yaml`) so IK/workspace accuracy can
  be re-verified.
- One-click launcher, symlink install, desktop shortcut, and
  `git pull`-based updates — all under `v5`.

## Install / run / update

Everything lives in [`custom_sortingv5/`](custom_sortingv5/). See
[`custom_sortingv5/INSTALL.md`](custom_sortingv5/INSTALL.md) for the full
guide.

One-paste install (inside the Hiwonder container):

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/calo-lategan/new-repo2/main/custom_sortingv5/install.sh) --sudoers
```

Update later (no rebuild for Python edits):

```bash
git -C ~/jetarm_v5_src pull
```

Then launch from the **JetArm Sort v5** desktop shortcut, or:

```bash
~/jetarm_v5/launch_v5.sh
```

## Other folders

- `full jetarm source for context src/` — the stock Hiwonder ROS 2 source,
  kept for reference (the v5 launch reuses its AprilTag `calibration` node).
- `example pi scripts/` — assorted reference scripts.
