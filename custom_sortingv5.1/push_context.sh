#!/usr/bin/env bash
# push_context.sh - push the FULL JetArm device context to the GitHub
# 'device-context' branch, under one top folder: FULL_jetarm/.
#
# It captures ALL READABLE content (source, configs, launches, XML/YAML/JSON,
# logs) from every folder, PLUS a COMPLETE MANIFEST of the entire tree - every
# file and its size, INCLUDING the binaries - so nothing is hidden. You see the
# whole layout; I can ask for any specific binary if it ever matters.
#
# Why binaries' CONTENT is excluded (engine/.so/.pt/weights/build): GitHub
# hard-rejects any file > 100 MB, a couple-GB branch permanently bloats the repo,
# and those files are unreadable to an analyst anyway. The manifest proves
# nothing is missing.
#
# PREREQ - git push auth (no gh needed, use a PAT):
#   Make a fine-grained token (repo new-repo2, Contents: Read+write), then run:
#     REPO_URL="https://calo-lategan:github_pat_XXXX@github.com/calo-lategan/new-repo2.git" \
#       bash ~/jetarm_v5_src/custom_sortingv5.1/push_context.sh
set -u

REPO_URL="${REPO_URL:-https://github.com/calo-lategan/new-repo2.git}"
BRANCH="${BRANCH:-device-context}"
export GIT_TERMINAL_PROMPT=0

WORK="$HOME/.jetarm_ctx_tmp"
rm -rf "$WORK"; mkdir -p "$WORK/payload/FULL_jetarm"
trap 'rm -rf "$WORK"' EXIT
DEST="$WORK/payload/FULL_jetarm"

# Folders to capture (relative to $HOME).
INCLUDE=( ros2_ws/src third_party_ros2 factory_utils jetarm_v5_profiles jetarm_v5_1 jetarm_v5/logs large_models )

echo "[ctx] 1/4 manifest of the ENTIRE tree (every file + size, incl. binaries)..."
{
  echo "# FULL JetArm tree manifest  (size_bytes  path)"; date; echo
  for d in "${INCLUDE[@]}"; do
    [ -e "$HOME/$d" ] && find "$HOME/$d" -printf '%11s  %p\n' 2>/dev/null
  done
} > "$DEST/MANIFEST_full_tree.txt"
{
  echo "# Top-level home folders + sizes (so you can see what else exists)"; echo
  du -sh "$HOME"/* 2>/dev/null | sort -rh
} > "$DEST/MANIFEST_home_overview.txt"
{
  echo "# Files >= 5 MB (the heavy binaries whose CONTENT was NOT included)"; echo
  for d in "${INCLUDE[@]}"; do
    [ -e "$HOME/$d" ] && find "$HOME/$d" -type f -size +5M -printf '%11s  %p\n' 2>/dev/null
  done | sort -rn
} > "$DEST/MANIFEST_big_files.txt"

echo "[ctx] 2/4 copying all READABLE content (no binaries/build)..."
tar -chf - -C "$HOME" \
  --ignore-failed-read \
  --exclude='build' --exclude='install' --exclude='log' \
  --exclude='__pycache__' --exclude='.git' --exclude='.cache' \
  --exclude='*.engine' --exclude='*.onnx' --exclude='*.pt' --exclude='*.pth' \
  --exclude='*.trt' --exclude='*.bin' --exclude='*.weights' --exclude='*.pb' \
  --exclude='*.so' --exclude='*.so.*' --exclude='*.a' --exclude='*.o' --exclude='*.lib' --exclude='*.whl' \
  --exclude='*.mp4' --exclude='*.avi' --exclude='*.mkv' --exclude='*.bag' \
  --exclude='*.zip' --exclude='*.tar' --exclude='*.tar.gz' --exclude='*.tgz' --exclude='*.7z' \
  "${INCLUDE[@]}" 2>/dev/null | tar -xf - -C "$DEST" 2>/dev/null

echo "[ctx]   payload size: $(du -sh "$WORK/payload" 2>/dev/null | cut -f1)"
# Guard against an accidental single huge text file that GitHub would reject.
big=$(find "$DEST" -type f -size +95M 2>/dev/null | head -1)
if [ -n "$big" ]; then
  echo "[ctx] WARN: $big is >95 MB - GitHub may reject it. Removing it from the payload."
  find "$DEST" -type f -size +95M -delete 2>/dev/null
fi

echo "[ctx] 3/4 cloning repo (shallow)..."
if ! git clone --depth 1 "$REPO_URL" "$WORK/repo" 2>&1 | tail -1; then
  echo "[ctx] clone failed"; exit 1
fi
cd "$WORK/repo" || exit 1
git checkout --orphan "$BRANCH" 2>/dev/null || git checkout -B "$BRANCH"
git rm -rf . >/dev/null 2>&1 || true
cp -a "$WORK/payload/." .
git add -A
git -c user.email='jetarm@local' -c user.name='jetarm' \
    commit -q -m "FULL_jetarm context snapshot $(date +%F_%H-%M-%S)" \
  || { echo "[ctx] nothing to commit"; exit 0; }

echo "[ctx] 4/4 pushing to origin/$BRANCH (force-replace)..."
if git push -f origin "$BRANCH"; then
  echo
  echo "[ctx] DONE -> https://github.com/calo-lategan/new-repo2/tree/$BRANCH"
  echo "[ctx] Nothing kept on the Jetson (temp dir removed)."
else
  echo
  echo "[ctx] PUSH FAILED - the device has no push credentials."
  echo "[ctx] Re-run with a PAT in the URL (see the header of this script)."
  exit 4
fi
