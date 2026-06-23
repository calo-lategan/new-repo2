#!/usr/bin/env bash
# tools/push_logs.sh - copy the JetArm v5 session logs into the repo's
# logs/ folder, commit, and push so Claude Code sessions can read them.
#
# Usage on the Jetson:
#   bash ~/new-repo2/tools/push_logs.sh         # default repo path
#   REPO=/path/to/repo bash tools/push_logs.sh  # override
#
# Env:
#   REPO       path to the cloned repo  (default: ~/new-repo2)
#   LOG_DIR    where the node writes logs (default: ~/jetarm_v5/logs)
#   KEEP       number of most recent session logs to copy (default: 5)
#   BRANCH     branch to push to (default: main)
#   MAX_BYTES  truncate any single log file above this size (default: 1048576 = 1 MB)
#
# Never aborts the session - errors are reported but the script always
# attempts the push.

set -u

REPO="${REPO:-$HOME/new-repo2}"
LOG_DIR="${LOG_DIR:-$HOME/jetarm_v5/logs}"
KEEP="${KEEP:-5}"
BRANCH="${BRANCH:-main}"
MAX_BYTES="${MAX_BYTES:-1048576}"

ts=$(date '+%Y-%m-%d_%H-%M-%S')

if [ ! -d "$REPO/.git" ]; then
  echo "[push_logs] not a git repo: $REPO" >&2
  exit 1
fi

if [ ! -d "$LOG_DIR" ]; then
  echo "[push_logs] log dir not found: $LOG_DIR (node hasn't logged yet?)" >&2
  exit 0
fi

cd "$REPO" || exit 1
mkdir -p "$REPO/logs"

echo "[push_logs] pulling latest..."
git pull --rebase --autostash origin "$BRANCH" || \
  echo "[push_logs] git pull failed (continuing)"

# Copy the KEEP most-recent session files. Truncate any single file above
# MAX_BYTES (keep first half + last half so head and tail context survive).
copied=0
files=$(ls -1t "$LOG_DIR"/*.log 2>/dev/null | head -n "$KEEP")
for src in $files; do
  base=$(basename "$src")
  dst="$REPO/logs/$base"
  size=$(stat -c %s "$src" 2>/dev/null || echo 0)
  if [ "$size" -gt "$MAX_BYTES" ]; then
    half=$((MAX_BYTES / 2))
    {
      echo "# [push_logs] file truncated; original size=$size bytes";
      head -c "$half" "$src";
      echo;
      echo "# ... [truncated] ...";
      tail -c "$half" "$src";
    } > "$dst"
  else
    cp -f "$src" "$dst"
  fi
  copied=$((copied + 1))
done

if [ "$copied" -eq 0 ]; then
  echo "[push_logs] no logs found in $LOG_DIR"
  exit 0
fi

git add logs/
if git diff --cached --quiet; then
  echo "[push_logs] no log changes to commit"
  exit 0
fi

git commit -m "logs: session $ts ($copied file(s))" \
  -m "Auto-published by tools/push_logs.sh from the JetArm."
if ! git push -u origin "$BRANCH"; then
  for sleep_for in 2 4 8 16; do
    sleep "$sleep_for"
    if git push -u origin "$BRANCH"; then
      break
    fi
  done
fi

echo "[push_logs] done."
