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

# Never let git block on an interactive credential/host prompt. Without push
# credentials on the device, `git push` over https would otherwise HANG waiting
# for a username/password on a stdin that the UI subprocess never provides -
# stalling until the 120 s timeout and freezing the tuner. These make a missing
# credential fail FAST with a clear error instead (rc=4), and the device-auth
# setup is in UPDATE_JETARM.md.
export GIT_TERMINAL_PROMPT=0
export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new}"

# Round 13 R13.2: self-locate the repo by walking up from this script's
# own path, so a manual run without REPO= still works regardless of
# where the repo was cloned (~/jetarm_v5_src on the JetArm installer).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO:-$(cd "$SCRIPT_DIR/.." && pwd)}"
LOG_DIR="${LOG_DIR:-$HOME/jetarm_v5/logs}"
KEEP="${KEEP:-5}"
# Round 16 MM.1: push logs to a DEDICATED branch, never main. This avoids
# non-fast-forward rejections against a moving main, and keeps log noise
# out of the main history. The device's update flow (reset --hard
# origin/main) cleans the local log commit; the logs live on this branch.
BRANCH="${BRANCH:-jetarm-logs}"
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

# NOTE: we deliberately do NOT pull here. Logs go to a dedicated branch
# via `git push HEAD:$BRANCH`, so the local checkout (usually on main) is
# left undisturbed. The working-tree copy under logs/ survives until the
# next `reset --hard`, and the pushed copy lives on origin/$BRANCH.

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
  echo "[push_logs] no logs found in $LOG_DIR" >&2
  # Distinct rc so the UI can show "no new logs" instead of "pushed".
  exit 2
fi

git add logs/
if git diff --cached --quiet; then
  echo "[push_logs] no log changes to commit (logs already in repo)" >&2
  exit 3
fi

git commit -m "logs: session $ts ($copied file(s))" \
  -m "Auto-published by tools/push_logs.sh from the JetArm." >/dev/null

# Push the current commit to the dedicated logs branch (HEAD:$BRANCH), so
# the local checkout's branch (main) is irrelevant and never conflicts.
# Round 16 MM.2: capture stderr and echo the last line so the UI can show
# WHY a push failed (auth vs rejected vs network).
#
# AUTH (Round 20h): the device has NO stored git credentials - proven by
# origin/jetarm-logs never existing while origin/device-context does
# (push_context.sh works only because a PAT is embedded in REPO_URL at
# invocation). Plain `git push origin` therefore always failed here. If a
# token file exists, push to an authenticated URL instead - the same
# mechanism push_context.sh uses, made persistent and button-friendly:
#   echo 'github_pat_XXXX' > ~/.jetarm_v5.pat && chmod 600 ~/.jetarm_v5.pat
PAT_FILE="${PAT_FILE:-$HOME/.jetarm_v5.pat}"
PUSH_TARGET="origin"
tok=""
if [ -f "$PAT_FILE" ]; then
  tok="$(head -1 "$PAT_FILE" | tr -d '[:space:]')"
  if [ -n "$tok" ]; then
    PUSH_TARGET="https://calo-lategan:${tok}@github.com/calo-lategan/new-repo2.git"
    echo "[push_logs] using token from $PAT_FILE"
  fi
fi
push_ok=0
push_err=""
for attempt in 1 2 3 4 5; do
  push_err="$(git push "$PUSH_TARGET" "HEAD:$BRANCH" 2>&1)"
  rc=$?
  # Never let the token leak into logs/UI via git's error text (it echoes
  # the remote URL on failure).
  if [ -n "$tok" ]; then
    push_err="${push_err//$tok/***}"
  fi
  if [ $rc -eq 0 ]; then
    push_ok=1
    break
  fi
  echo "[push_logs] push attempt $attempt failed: $push_err" >&2
  sleep $((attempt * 2))
done

# Round 14 DD.3: surface real rc so the UI doesn't claim success on a
# silent failure. Previously the script exited 0 even when every push
# attempt failed, so the UI lied.
if [ "$push_ok" -ne 1 ]; then
  echo "[push_logs] git push failed after retries. Last error:" >&2
  echo "$push_err" | tail -n 1 >&2
  echo "[push_logs] (no credentials: echo your PAT into ~/.jetarm_v5.pat (chmod 600) and retry - see UPDATE_JETARM.md)" >&2
  exit 4
fi

echo "[push_logs] done - pushed to origin/$BRANCH"
