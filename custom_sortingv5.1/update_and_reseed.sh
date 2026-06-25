#!/usr/bin/env bash
# update_and_reseed.sh - rebuild/redeploy v5.1 AND reseed the boot YAMLs to the
# factory baseline, while PRESERVING your model settings (engine path/task + the
# YOLO knobs). Does NOT touch transform.yaml / calibration.yaml (your AprilTag /
# kinematics calibration) - re-run a clean CALIBRATE after.
#
# Run AFTER pulling the latest code:
#   git -C ~/jetarm_v5_src fetch origin && git -C ~/jetarm_v5_src reset --hard origin/main
#   bash ~/jetarm_v5_src/custom_sortingv5.1/update_and_reseed.sh
#   ~/jetarm_v5_1/launch_v5.sh
set -u

SRC="$HOME/jetarm_v5_src/custom_sortingv5.1"
PROF="$HOME/jetarm_v5_profiles"
DEF="$PROF/default.yaml"
# Model settings to PRESERVE from your current config (everything else -> factory):
MODEL_KEYS="engine_path engine_task yolo_conf_thresh yolo_iou_thresh yolo_max_det inference_max_hz yolo_enabled_classes"

ts=$(date +%F_%H-%M-%S)
mkdir -p "$PROF"

# 0) Back up the CURRENT boot config FIRST, so we can read your model settings
#    back even if install.sh reseeds default.yaml.
SAVED="/tmp/default.yaml.preupdate.$ts"
if [ -f "$DEF" ]; then
  cp "$DEF" "$SAVED"; cp "$DEF" "$DEF.bak.$ts"
  echo "[reseed] backed up current default.yaml -> $DEF.bak.$ts"
else
  echo "[reseed] no existing $DEF (fresh install) - nothing to preserve"
fi

# 1) Rebuild + redeploy (colcon build for the entry points).
echo "[reseed] running install.sh ..."
bash "$SRC/install.sh"

# 2) Reseed default.yaml to the factory baseline (vendor colour grid, world=AprilTag,
#    min_descend 0.01, short-axis 0.30, offsets 0, scale 1.0, factory motion/grip).
cp "$SRC/profiles/default.yaml" "$DEF"
echo "[reseed] default.yaml reset to factory."

# 3) Restore YOUR model settings from the pre-update backup (line-preserving so the
#    factory file's format/comments stay intact).
if [ -f "$SAVED" ]; then
  for k in $MODEL_KEYS; do
    v=$(grep -E "^[[:space:]]*$k:" "$SAVED" | head -1 | sed -E "s|^[[:space:]]*$k:[[:space:]]*||")
    if [ -n "$v" ]; then
      sed -i -E "s|^([[:space:]]*$k:).*|\1 $v|" "$DEF"
      echo "[reseed] preserved model setting  $k: $v"
    fi
  done
fi

# 4) Reseed the speed presets too (motion-only; no model keys).
cp "$SRC/profiles/slow.yaml" "$SRC/profiles/medium.yaml" "$SRC/profiles/fast.yaml" "$PROF/" 2>/dev/null || true
echo "[reseed] slow/medium/fast presets reset to factory."

# 5) Verify the key values landed.
echo "[reseed] --- key values now in default.yaml ---"
grep -E "engine_path|engine_task|yolo_conf_thresh|yolo_iou_thresh|yolo_max_det|inference_max_hz|workspace_scale|grip_offset_|min_descend_z_m|grasp_short_axis_min_ratio|grab_depth|motion_speed|aggression|detection_avg_frames|place_positions" "$DEF"
echo
echo "[reseed] DONE. Backup of your previous config: $DEF.bak.$ts"
echo "[reseed] Next: ~/jetarm_v5_1/launch_v5.sh  -> restart_v5.sh -> run a clean AprilTag CALIBRATE."
