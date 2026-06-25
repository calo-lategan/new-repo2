#!/usr/bin/env bash
# push_context.sh - push the FULL JetArm device context to the GitHub
# 'device-context' branch, under one top folder: FULL_jetarm/.
#
# It copies every READABLE file (everything under 5 MB - all source, configs,
# launches, XML/YAML/JSON, logs) PLUS a COMPLETE MANIFEST of the entire tree
# (every file + size, INCLUDING the big binaries) so nothing is hidden. The
# 5 MB cap is a robust, extension-proof way to skip models/.so/datasets/build
# (all far bigger) - which are unreadable to an analyst and would blow past
# GitHub's 100 MB/file limit. The manifest proves nothing's missing.
#
# PREREQ - git push auth (no gh needed). Token must have Contents: Read+WRITE:
#   REPO_URL="https://calo-lategan:github_pat_XXXX@github.com/calo-lategan/new-repo2.git" \
#     bash ~/jetarm_v5_src/custom_sortingv5.1/push_context.sh
set -u

REPO_URL="${REPO_URL:-https://github.com/calo-lategan/new-repo2.git}"
BRANCH="${BRANCH:-device-context}"
MAXSZ="${MAXSZ:-5M}"           # per-file size cap for copied content
export GIT_TERMINAL_PROMPT=0

WORK="$HOME/.jetarm_ctx_tmp"
rm -rf "$WORK"; mkdir -p "$WORK/payload/FULL_jetarm"
trap 'rm -rf "$WORK"' EXIT
DEST="$WORK/payload/FULL_jetarm"

# Folders to capture (relative to $HOME).
INCLUDE=( ros2_ws/src third_party_ros2 factory_utils jetarm_v5_profiles jetarm_v5_1 jetarm_v5/logs large_models )

cd "$HOME" || exit 1

echo "[ctx] 1/4 manifest of the ENTIRE tree (every file + size, incl. binaries)..."
{
  echo "# FULL JetArm tree manifest  (size_bytes  path)"; date; echo
  for d in "${INCLUDE[@]}"; do
    [ -e "$HOME/$d" ] && find "$HOME/$d" -printf '%11s  %p\n' 2>/dev/null
  done
} > "$DEST/MANIFEST_full_tree.txt"
{ echo "# Top-level home folders + sizes"; echo; du -sh "$HOME"/* 2>/dev/null | sort -rh; } \
  > "$DEST/MANIFEST_home_overview.txt"
{
  echo "# Files >= $MAXSZ (heavy binaries whose CONTENT was NOT copied)"; echo
  for d in "${INCLUDE[@]}"; do
    [ -e "$HOME/$d" ] && find "$HOME/$d" -type f -size "+$MAXSZ" -printf '%11s  %p\n' 2>/dev/null
  done | sort -rn
} > "$DEST/MANIFEST_big_files.txt"

echo "[ctx] 2/4 copying readable files (< $MAXSZ each; skips models/.so/datasets/build)..."
find "${INCLUDE[@]}" -type f -size "-$MAXSZ" \
  -not -path '*/build/*' -not -path '*/install/*' \
  -not -path '*/__pycache__/*' -not -path '*/.git/*' -not -path '*/.cache/*' \
  -print0 2>/dev/null \
  | tar --null -cf - -T - 2>/dev/null \
  | tar -xf - -C "$DEST" 2>/dev/null
sz=$(du -sh "$WORK/payload" 2>/dev/null | cut -f1)
echo "[ctx]   payload size: ${sz:-?}  (if this is still >500M, set MAXSZ=1M and re-run)"

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
  echo; echo "[ctx] DONE -> https://github.com/calo-lategan/new-repo2/tree/$BRANCH"
  echo "[ctx] Nothing kept on the Jetson (temp dir removed)."
else
  echo; echo "[ctx] PUSH FAILED (403?) - the token needs Contents: Read AND WRITE."
  echo "[ctx] Fix the token's permission (or use a classic token with 'repo' scope) and re-run."
  exit 4
fi
