# Vendor Original — Sorting Defaults, Drop Locations & World Map (source of truth)

Exact values extracted verbatim from the **untouched** Hiwonder JetArm vendor
source (`full jetarm source for context src/`). Use this to confirm/restore
known-good behaviour. Line refs are to the vendor files.

> **TL;DR — v5.1's baseline already matches the vendor.** The colour drop
> positions, gripper pulses (540 close / 200 open), `grab_depth` 0.02, IK pitch
> 80 are identical to the original. If sorting "changed", it's because the
> device's `~/jetarm_v5_profiles/default.yaml` accumulated UI tweaks
> (`workspace_scale`, `grip_offset_*`, the slow profile, depth params). Re-seed
> that file (below) to return to this baseline.

---

## 1. Drop locations (metres, world frame `[x, y, z]`)

### Colour sorting — `app/app/object_sorting.py:33-38` (`place_position` dict)
| class | [x, y, z] |
|-------|-----------|
| green | `[-0.006, 0.23, 0.015]` |
| red   | `[ 0.064, 0.23, 0.015]` |
| blue  | `[-0.076, 0.23, 0.015]` |
| tag1  | `[-0.076, 0.16, 0.015]` |
| tag2  | `[-0.006, 0.16, 0.015]` |
| tag3  | `[ 0.064, 0.16, 0.015]` |

Indexed by the class label string: `place_position[target[0]]` (`:357`).
**These are exactly the v5.1 `place_positions` defaults** (scaff→`[-0.076,0.16]`,
red/green/blue cubes at y=0.23, etc.).

### Waste sorting — `app/app/waste_classification.py:39-42` (the live, authoritative table)
| category | [x, y, z] |
|----------|-----------|
| residual_waste  | `[ 0.087, -0.225, 0.02]` |
| food_waste      | `[ 0.025, -0.225, 0.02]` |
| hazardous_waste | `[-0.036, -0.225, 0.02]` |
| recyclable_waste| `[-0.098, -0.225, 0.02]` |

(Two other waste tables exist — `lab_config.yaml` `waste_target_position` and
`positions.yaml` `target_1..4` — but are **not** used by the placing logic.)

---

## 2. Pick choreography — `app/app/utils/pick_and_place.py` (interpolation=False)
Call site `object_sorting.py:355`: `pick(position, 80, yaw, 540, 0.02, ...)`
→ pitch **80**, close pulse **540** (servo 10), `gripper_depth` **0.02**.

| step | Z change | move s | dwell s | line |
|------|----------|--------|---------|------|
| pre-transport bump (if `x>0.22`) | `+0.01` | — | — | object_sorting.py:345 |
| approach hover | `+0.01` | base 1.0 / arm 1.0 | 1 / 1 | pick_and_place.py:30,48,51 |
| wrist yaw align | — | 0.5 | 0.5 | :54 |
| **descend to grab** | `-(0.01 + gripper_depth)` = **-0.03** | 0.5 | 0.5 | :58,71 |
| close gripper → 540 | — | 0.5 | 0.5 | :74 |
| lift after grab | `+(0.02 + gripper_depth)` = **+0.04** | 0.5 | 0.5 | :78,91 |

**Net descend Z = `Z_target − gripper_depth`** (= `Z_target − 0.02`). With the
forced pick height `Z_target = 0.03`, the grab descends to **0.01** (reachable).
Net lift = `Z_target + 0.02`.

Carry/retreat pose (`pick_and_place.py:115/117`, branches on `CHASSIS_TYPE`):
- `Slide_Rails`: `[0.11, 0.0, 0.15]`, pitch 73
- other: `[0.11, 0.0, 0.09]`, pitch 73

---

## 3. Place choreography — `app/app/utils/pick_and_place.py:127-197`
Call site `object_sorting.py:378`: `place(position, 80, yaw, 200, ...)`
→ pitch **80**, release/open pulse **200** (servo 10). IK pitch range `[-90,90]`.

| step | Z change | move s | dwell s | line |
|------|----------|--------|---------|------|
| raise above drop | `+0.03` | base 1.0 / arm 1.0 | 1 / 1 | :130,149,153 |
| wrist yaw to place | — | 0.5 | 0.8 | :161 |
| lower to drop Z | `-0.03` | 1.0 | 1.2 | :159,175 |
| release → 200 | — | 0.5 | 0.5 | :178 |
| retreat | `+0.03` | 0.5 | 0.5 | :181,193 |
| reset gripper → 200 | — | 0.5 | — | :195 |

---

## 4. Motion params
| item | value | line |
|------|-------|------|
| IK pitch (pick/place) | `80` | object_sorting.py:355,378 |
| IK pitch range (approach/grab/place) | `[-90.0, 90.0]` | pick_and_place.py |
| IK pitch range (final lift) | `[-180.0, 180.0]` | pick_and_place.py:81 |
| gripper close (pick) | `540` | object_sorting.py:355 |
| gripper open/release (place, home) | `200` | object_sorting.py:378, 213 |
| `gripper_depth` (grab depth) | `0.02` | object_sorting.py:355 |
| pick yaw pulse | `500 + int(result[0]/240*1000)` | object_sorting.py:467 |
| home/observe joints (servos 1-5) | `[500, 520, 210, 50, 500]`, gripper 200 | object_sorting.py:161,213 |
| world-position height (Z forced) | `0.03` | object_sorting.py:294,300 |
| stillness gate | `e_distance <= 0.005` | object_sorting.py:452 |
| count_still / count_move trigger | `> 10` | object_sorting.py:460,462 |
| tag_size | `0.025` | object_sorting.py:65 |
| far-reach z bump (`x>0.22`) | `+0.01` | object_sorting.py:345 |

### v5.1 equivalents (already at these values in the repo baseline)
`gripper_close_pulse: 540`, `gripper_open_pulse: 200`, `grab_depth: 0.02`,
pick/place pitch `80`, `place_positions` = the colour table above,
`grip_offset_x/y/z: 0.0`, `workspace_scale: 1.0`.

---

## 5. World-map math (how a pixel becomes a world coordinate)

`object_sorting.py:294-308` (`get_object_world_position`):
1. Build projection `[[R|t],[0 0 0 1]]` from the AprilTag extrinsic (`extristric`).
2. `pixels_to_world` back-projects the pixel onto the table plane (single-plane
   homography; **no workspace gain here**). The extrinsic plane is lifted
   `+0.03 m` first (`extristric_plane_shift`).
3. Negate world X and Y (tag frame vs arm frame).
4. `position = white_area_center[:3,3] + world_pose` — add the world origin.
5. `position[2] = height` (**0.03**, Z is NOT from vision).
6. Apply `calibration.yaml['pixel']` scale/offset.
7. In transport, apply `calibration.yaml['kinematics']` scale/offset (and the
   place uses the angle-branched form).

**Important:** the world map is the AprilTag calibration **as the base**, refined
by two small per-axis fudge layers from `calibration.yaml` (`pixel.*` and
`kinematics.*`). There is **no `workspace_scale`** in the vendor — that param is
v5-only and should stay **1.0**. The vendor's equivalent gains are
`pixel.scale`/`kinematics.scale` in `calibration.yaml`, which the AprilTag flow
manages — leave them to calibration.

### World origin — `app/config/transform.yaml` `white_area_pose_world` (identity rotation)
`X = 0.1655103`, `Y = 0.0090076`, `Z = -0.0033023`  (the white-area centre in the
arm frame; **written by AprilTag calibration**, do not hand-edit).

### Calibration affines — `app/config/calibration.yaml`
```
kinematics:  scale [1.0, 1.05, 1.0]   offset [0.003, 0.006, 0.0]
pixel:       scale [1.0, 0.97, 1.0]   offset [0.009, -0.006, 0.0]
depth:       scale [1.0, 1.0,  1.0]   offset [0.022, -0.010, 0.0]   (unused by sorting)
```
### Device (Slide_Rails) — `stepper/config/calibration.yaml` (what your arm actually loads)
```
kinematics:  scale [1.0, 1.07, 1.0]   offset [0.0,  0.0,    0.0]
pixel:       scale [1.0, 1.0,  1.0]   offset [0.0, -0.005, -0.005]
```
> Note: the device's `pixel.offset[2] = -0.005` means the forced 0.03 pick height
> reads as **0.025** in the descend log — that is normal. The grab only fails when
> `grab_depth` (or a negative `grip_offset_z`) drives `0.025 − grab_depth` below
> the arm's reach. Keep `grab_depth = 0.02` and `grip_offset_z = 0.0`.

---

## 6. Restore your device to this baseline
1. Reset the boot config (re-seed the clean `default.yaml`):
   ```bash
   git -C ~/jetarm_v5_src fetch origin && git -C ~/jetarm_v5_src reset --hard origin/main
   cp ~/jetarm_v5_src/custom_sortingv5.1/profiles/default.yaml ~/jetarm_v5_profiles/default.yaml
   ```
   (or just re-run `install.sh`, which re-seeds the profiles.)
2. Confirm in `~/jetarm_v5_profiles/default.yaml`: `workspace_scale: 1.0`,
   `grip_offset_x/y/z: 0.0`, `grab_depth: 0.02`, `motion_speed: 1.5`,
   `aggression: 1.3`.
3. Relaunch, then run **one clean AprilTag calibration** (tag flat, fully in
   view, steady). Trust it only if the plane normal is ~`[0, 0, -1]`.
4. Tune *small* from there: `grip_offset_x` for left/right, `grip_offset_z` only
   if the grab is consistently off in height.
