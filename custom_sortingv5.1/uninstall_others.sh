#!/usr/bin/env bash
# JetArm Sort - uninstall every non-v5 version from this JetArm.
#
# v5 is the single, current version. If v2 / v4 / v4.1 were ever installed
# on this Jetson, their symlinks + setup.py entry_points + launcher dirs +
# desktop shortcuts + sudoers stick around and break a clean v5 install.
# install.sh runs this cleanup automatically as a pre-step. This wrapper
# lets you run JUST the cleanup, without re-running the full v5 install,
# e.g. if you already have v5 working and just want the old versions gone.
#
# Run it from the v5 checkout:
#   bash ~/jetarm_v5_src/custom_sortingv5/uninstall_others.sh
#
# Args (optional):
#   --sudoers       also remove /etc/sudoers.d/jetarm-v{2,4,4.1} (needs sudo).
#   --dry-run       print "[would remove]" lines without deleting anything.
#   --keep-others   no-op (accepted for symmetry with install.sh; nothing to do).
#
# Env overrides (same as install.sh):
#   WS_DIR=$HOME/ros2_ws
#   APP_PKG=$WS_DIR/src/app
#
# Idempotent: safe to re-run; prints "nothing to remove" once clean.

set -e

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
APP_PKG="${APP_PKG:-$WS_DIR/src/app}"

DO_SUDOERS=0
DO_DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --sudoers)     DO_SUDOERS=1 ;;
        --dry-run)     DO_DRY_RUN=1 ;;
        --keep-others)
            echo "[uninstall] --keep-others: nothing to do (this script's only job is removal). Exiting."
            exit 0
            ;;
        --help|-h) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "unknown arg: $arg"; exit 1 ;;
    esac
done

if [ "$DO_SUDOERS" = "1" ] && ! sudo -n true 2>/dev/null; then
    # Pre-prompt for sudo so the per-version steps don't surprise the user
    # mid-run. They asked for it explicitly.
    sudo -v
fi

# Reuse install.sh's cleanup helpers. The helpers live in section 1b
# (between the marker comments install.sh keeps stable). Section 1c
# contains the interactive dispatch logic - we deliberately STOP before
# that and call do_cleanup_all ourselves, unconditionally.

HERE="$(cd "$(dirname "$0")" && pwd)"
INSTALL_SH="$HERE/install.sh"
if [ ! -f "$INSTALL_SH" ]; then
    echo "[uninstall] cannot find $INSTALL_SH next to this script" >&2
    exit 1
fi

TMP="$(mktemp)"
awk '
  /^# --- 1b\. Device cleanup:/                    { capture=1 }
  /^# --- 1c\. Device cleanup: resolve mode/       { capture=0 }
  capture                                          { print }
' "$INSTALL_SH" > "$TMP"

stage() { printf "\033[1;36m[uninstall]\033[0m %s\n" "$*"; }
ok()    { printf "\033[1;32m[uninstall]\033[0m %s\n" "$*"; }
err()   { printf "\033[1;31m[uninstall]\033[0m %s\n" "$*" >&2; }

# shellcheck disable=SC1090
source "$TMP"
rm -f "$TMP"

if [ "$DO_DRY_RUN" = "1" ]; then
    DRY_RUN=1 do_cleanup_all
    ok "dry run complete - nothing was removed"
else
    do_cleanup_all
    ok "device cleanup done. v5 itself is untouched - run install.sh to (re)install v5."
fi
