#!/usr/bin/env bash
# push_context.sh - snapshot the device context straight to the GitHub
# 'device-context' branch, so it never lives as a big file on the laptop or the
# Jetson. Uses a temp dir under $HOME that is DELETED on exit (only transient
# space; nothing permanent stays on the Jetson).
#
# PREREQUISITE (one time) - git must be able to PUSH:
#     gh auth login        # GitHub.com -> HTTPS -> your account
#     gh auth setup-git
#   (no gh? use a token instead:)
#     git config --global credential.helper store
#     # then the first push will ask for username + a GitHub PAT as the password
#
# RUN:
#     bash ~/jetarm_v5_src/custom_sortingv5.1/push_context.sh
#
# Lean by default (FULL source + configs + launches + profiles + logs, no
# binaries). Set LIGHT=1 to push ONLY the small device-specific deltas
# (profiles + live configs + launches + logs) and skip the big source trees.
set -u

REPO_URL="${REPO_URL:-https://github.com/calo-lategan/new-repo2.git}"
BRANCH="${BRANCH:-device-context}"
export GIT_TERMINAL_PROMPT=0

WORK="$HOME/.jetarm_ctx_tmp"
rm -rf "$WORK"; mkdir -p "$WORK/payload"
trap 'rm -rf "$WORK"' EXIT

# What to include (relative to $HOME).
INCLUDE=( ros2_ws/src jetarm_v5_profiles jetarm_v5_1 jetarm_v5/logs )
if [ "${LIGHT:-0}" != "1" ]; then
  INCLUDE+=( factory_utils third_party_ros2 )
fi

echo "[ctx] collecting lean context (no binaries/build): ${INCLUDE[*]}"
tar -chf - -C "$HOME" \
  --ignore-failed-read \
  --exclude='build' --exclude='install' --exclude='log' \
  --exclude='__pycache__' --exclude='.git' \
  --exclude='*.engine' --exclude='*.onnx' --exclude='*.pt' --exclude='*.pth' \
  --exclude='*.trt' --exclude='*.bin' --exclude='*.weights' \
  --exclude='*.so' --exclude='*.a' --exclude='*.o' \
  --exclude='*.mp4' --exclude='*.avi' --exclude='*.bag' \
  "${INCLUDE[@]}" 2>/dev/null | tar -xf - -C "$WORK/payload" 2>/dev/null
echo "[ctx] payload size: $(du -sh "$WORK/payload" 2>/dev/null | cut -f1)"

echo "[ctx] cloning repo (shallow)..."
if ! git clone --depth 1 "$REPO_URL" "$WORK/repo" 2>&1 | tail -1; then
  echo "[ctx] clone failed"; exit 1
fi
cd "$WORK/repo" || exit 1

# Fresh orphan branch so the snapshot is just the context (no repo history).
git checkout --orphan "$BRANCH" 2>/dev/null || git checkout -B "$BRANCH"
git rm -rf . >/dev/null 2>&1 || true
cp -a "$WORK/payload/." .
{ echo "# Device context snapshot"; date; echo; echo "Folders:"; ls -1; } > _CONTEXT_README.md
git add -A
git -c user.email='jetarm@local' -c user.name='jetarm' commit -q -m "device context snapshot $(date +%F_%H-%M-%S)" \
  || { echo "[ctx] nothing to commit"; exit 0; }

echo "[ctx] pushing to origin/$BRANCH (force-replace)..."
if git push -f origin "$BRANCH"; then
  echo
  echo "[ctx] DONE -> https://github.com/calo-lategan/new-repo2/tree/$BRANCH"
  echo "[ctx] Nothing kept on the Jetson (temp dir removed)."
else
  echo
  echo "[ctx] PUSH FAILED - the device has no push credentials yet."
  echo "[ctx] Run once:  gh auth login && gh auth setup-git   (or set up a PAT), then re-run."
  exit 4
fi
