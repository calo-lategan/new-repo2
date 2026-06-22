#!/usr/bin/env bash
# JetArm Sort v5 - one-shot installer for the Hiwonder container.
#
# What it does (idempotent - safe to re-run):
#   1. Clones/refreshes this repo into ~/jetarm_v5_src
#   2. Symlinks the v4 python files into ~/ros2_ws/src/app/app/
#   3. Symlinks the v4 launch file into ~/ros2_ws/src/app/launch/
#   4. Adds the v4 console_scripts entries to ~/ros2_ws/src/app/setup.py
#      (only if they're not already there)
#   5. Seeds ~/jetarm_v5_profiles/ with default.yaml + yolo.yaml (sample)
#   6. Installs ~/jetarm_v5/launch_v5.sh + image_view_chain.sh + re-enable-factory.sh
#   7. Installs the desktop shortcut to ~/Desktop and ~/.local/share/applications
#   8. Runs `colcon build --packages-select app --symlink-install`
#   9. (optional, with --sudoers) writes a NOPASSWD rule for the systemctl
#      restart so the desktop shortcut is truly one-click.
#  10. Prints clear next-step instructions.
#
# Why symlinks: a future `git -C ~/jetarm_v5_src pull` is immediately picked
# up by the next `ros2 launch` - no copy, no rebuild for pure-Python edits.
#
# This is designed to be run inside the Hiwonder container. From the host
# you can either ssh / lxc-attach into the container and run the curl one-
# liner from INSTALL.md, or run this script from a checkout you already
# have.
#
# Usage:
#   ./install.sh                  # default install (interactive prompt if
#                                 #   prior v2/v4/v4.1 installs detected)
#   ./install.sh --sudoers        # also install the NOPASSWD sudoers rule
#   ./install.sh --no-build       # skip colcon build (do it yourself)
#   ./install.sh --keep-others    # install v5 alongside v2/v4/v4.1 (no prompt)
#   ./install.sh --clean-others   # remove every non-v5 install (no prompt)
#   REPO=...  BRANCH=main  ./install.sh
#   INSTALL_MODE=keep ./install.sh   # same as --keep-others (curl-pipe friendly)
#   INSTALL_MODE=clean ./install.sh  # same as --clean-others
#
# Env overrides:
#   REPO=https://github.com/calo-lategan/new-repo2.git
#   BRANCH=main
#   SRC_DIR=$HOME/jetarm_v5_src
#   WS_DIR=$HOME/ros2_ws
#   PROFILES_DIR=$HOME/jetarm_v5_profiles
#   LAUNCHER_DIR=$HOME/jetarm_v5

set -e

REPO="${REPO:-https://github.com/calo-lategan/new-repo2.git}"
BRANCH="${BRANCH:-main}"
SRC_DIR="${SRC_DIR:-$HOME/jetarm_v5_src}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
PROFILES_DIR="${PROFILES_DIR:-$HOME/jetarm_v5_profiles}"
LAUNCHER_DIR="${LAUNCHER_DIR:-$HOME/jetarm_v5}"
APP_PKG="${APP_PKG:-$WS_DIR/src/app}"
DO_SUDOERS=0
DO_BUILD=1
KEEP_OTHERS=0
CLEAN_OTHERS=0

for arg in "$@"; do
    case "$arg" in
        --sudoers)       DO_SUDOERS=1 ;;
        --no-build)      DO_BUILD=0 ;;
        --keep-others)   KEEP_OTHERS=1 ;;
        --clean-others)  CLEAN_OTHERS=1 ;;
        --help|-h)
            sed -n '2,36p' "$0"; exit 0 ;;
        *) echo "unknown arg: $arg"; exit 1 ;;
    esac
done

if [ "$KEEP_OTHERS" = "1" ] && [ "$CLEAN_OTHERS" = "1" ]; then
    echo "--keep-others and --clean-others are mutually exclusive" >&2
    exit 1
fi

stage() { printf "\033[1;36m[install]\033[0m %s\n" "$*"; }
ok()    { printf "\033[1;32m[install]\033[0m %s\n" "$*"; }
err()   { printf "\033[1;31m[install]\033[0m %s\n" "$*" >&2; }

# --- 0. Sanity ------------------------------------------------------------

stage "JetArm Sort v5 installer"
stage "  repo     : $REPO ($BRANCH)"
stage "  src dir  : $SRC_DIR"
stage "  ws dir   : $WS_DIR"
stage "  app pkg  : $APP_PKG"
stage "  profiles : $PROFILES_DIR"
stage "  launcher : $LAUNCHER_DIR"

if [ ! -d "$WS_DIR" ]; then
    err "$WS_DIR not found. This script must run inside the Hiwonder container."
    err "Inside the container, $HOME/ros2_ws should exist."
    exit 1
fi
if [ ! -d "$APP_PKG" ]; then
    err "App package $APP_PKG not found - is this really the Hiwonder image?"
    exit 1
fi
if ! command -v git >/dev/null 2>&1; then
    err "git missing - apt-get install -y git"
    exit 1
fi

# --- 1. Clone / refresh source -------------------------------------------

if [ -d "$SRC_DIR/.git" ]; then
    stage "refreshing existing checkout in $SRC_DIR"
    (cd "$SRC_DIR" && git fetch --depth=1 origin "$BRANCH" && git checkout "$BRANCH" && git reset --hard "origin/$BRANCH")
else
    stage "cloning $REPO into $SRC_DIR"
    git clone --depth=1 -b "$BRANCH" "$REPO" "$SRC_DIR"
fi
V4="$SRC_DIR/custom_sortingv5"
if [ ! -d "$V4" ]; then
    err "v4 folder not found inside the checkout: $V4"
    exit 1
fi
ok "source ready"

# --- 1b. Device cleanup: helpers (uninstall every non-v5 version) -------
#
# v5 is the single, current version. If v2 / v4 / v4.1 ever got installed
# on this Jetson, their symlinks + entry_points + launcher dirs + desktop
# shortcuts + sudoers are still there and will fight v5 (or break colcon
# build when we go to compile, since their entry_points reference module
# files we never link in).
#
# These helpers are also sourced by uninstall_others.sh; keep them as
# bare functions in this 1b section (the dispatch logic lives in 1c).
#
# Order matters: remove the setup.py entry_points BEFORE deleting the
# source symlinks. Otherwise colcon build later fails because setup.py
# still references modules whose symlinks were just deleted.
#
# Idempotent: every step is "if it exists, remove it". Re-running prints
# "nothing to remove".
#
# Dry-run: when DRY_RUN=1, every destructive call is replaced by a
# "[would remove]" log line. Used to render the install-time preview
# before asking the user whether to keep or clean prior versions.

_destructive() {
    # _destructive "<description>" cmd args...
    # If DRY_RUN=1, log "[would remove] <description>" and skip the command.
    # Otherwise run the command and log "removed <description>".
    local desc="$1"; shift
    if [ "${DRY_RUN:-0}" = "1" ]; then
        stage "    [would remove] $desc"
    else
        "$@"
        stage "    removed $desc"
    fi
}

remove_entry_from_setup() {
    # $1 = setup.py path, $2 = entry-point string to remove
    local setup_path="$1" entry="$2"
    [ -f "$setup_path" ] || return 0
    if ! grep -qF "$entry" "$setup_path"; then
        return 0
    fi
    if [ "${DRY_RUN:-0}" = "1" ]; then
        stage "    [would remove] setup.py entry: $entry"
        return 0
    fi
    python3 - "$setup_path" "$entry" <<'PY'
import re, sys
path, entry = sys.argv[1], sys.argv[2]
text = open(path).read()
m = re.search(r"(\s*'console_scripts'\s*:\s*\[)(.*?)(\n\s*\])", text, re.S)
if not m:
    sys.stderr.write("could not find console_scripts list - leaving alone\n")
    sys.exit(0)
inner = m.group(2)
kept = []
for line in inner.split('\n'):
    if entry in line:
        continue
    kept.append(line)
new_inner = '\n'.join(kept)
out = text[:m.start()] + m.group(1) + new_inner + m.group(3) + text[m.end():]
open(path, 'w').write(out)
PY
    stage "    removed setup.py entry: $entry"
}

cleanup_old_version() {
    # $1 = label (v2/v4/v4.1)
    # $2 = node module basename (e.g. custom_sortingv4_1)
    # $3 = ui module basename (e.g. tune_uiv4_1)
    # $4 = launch file basename (e.g. custom_sorting_nodev4.1.launch.py)
    # $5 = launcher dir name (e.g. jetarm_v4_1)
    # $6 = desktop file stem (e.g. jetarm-sort-v4.1)
    # $7 = sudoers file name (e.g. jetarm-v4.1)
    # $8 = env file name in $HOME (e.g. .jetarm_v4_1.env)
    # $9 = src dir name (e.g. jetarm_v4_1_src) - NOT auto-deleted, only flagged
    local label="$1" node_mod="$2" ui_mod="$3" launch_file="$4"
    local launcher="$5" desktop_stem="$6" sudoers="$7" envfile="$8" srcdir="$9"
    local touched=0

    # 1) setup.py entry_points (must precede symlink removal)
    remove_entry_from_setup "$APP_PKG/setup.py" \
        "$node_mod = app.$node_mod:main" && touched=1
    remove_entry_from_setup "$APP_PKG/setup.py" \
        "$ui_mod = app.$ui_mod:main" && touched=1

    # 2) Source symlinks. Only delete if it IS a symlink (don't trash a
    #    user's actual file that happens to share a name).
    for f in "$APP_PKG/app/$node_mod.py" "$APP_PKG/app/$ui_mod.py" \
             "$APP_PKG/launch/$launch_file"; do
        if [ -L "$f" ]; then
            _destructive "symlink: $f" rm -f "$f"; touched=1
        fi
    done

    # 2b) Install-side launch symlinks. Previous colcon runs created these
    #     pointing into $APP_PKG/launch/*; once we delete the src-side link
    #     above, the install-side link is dangling and breaks the next
    #     `colcon build`. Remove the symlinks (never real files - a real
    #     file in install/ for an old version would only exist if someone
    #     manually broke colcon, in which case we shouldn't touch it).
    local install_launch="$WS_DIR/install/app/share/app/launch/$launch_file"
    if [ -L "$install_launch" ]; then
        _destructive "install-side symlink: $install_launch" \
            rm -f "$install_launch"
        touched=1
    fi

    # 3) Launcher dir
    if [ -d "$HOME/$launcher" ]; then
        _destructive "launcher dir: $HOME/$launcher" rm -rf "$HOME/$launcher"
        touched=1
    fi

    # 4) Desktop shortcuts (both locations)
    for f in "$HOME/Desktop/$desktop_stem.desktop" \
             "$HOME/.local/share/applications/$desktop_stem.desktop"; do
        if [ -f "$f" ]; then
            _destructive "desktop: $f" rm -f "$f"; touched=1
        fi
    done

    # 5) Sudoers - needs sudo. Skip silently if not allowed.
    if [ -f "/etc/sudoers.d/$sudoers" ]; then
        if [ "${DRY_RUN:-0}" = "1" ]; then
            stage "    [would remove] sudoers: /etc/sudoers.d/$sudoers"
            touched=1
        elif sudo -n true 2>/dev/null; then
            _destructive "sudoers: /etc/sudoers.d/$sudoers" \
                sudo rm -f "/etc/sudoers.d/$sudoers"
            touched=1
        else
            stage "    [skip] /etc/sudoers.d/$sudoers (need sudo - run with --sudoers)"
        fi
    fi

    # 6) Env file
    if [ -f "$HOME/$envfile" ]; then
        _destructive "env file: $HOME/$envfile" rm -f "$HOME/$envfile"
        touched=1
    fi

    # 7) Old source tree - FLAG ONLY (may contain user edits / profiles)
    if [ -d "$HOME/$srcdir" ]; then
        local sz; sz=$(du -sh "$HOME/$srcdir" 2>/dev/null | awk '{print $1}')
        stage "    [kept] $HOME/$srcdir ($sz) - delete manually if you don't need it: rm -rf $HOME/$srcdir"
    fi

    if [ "$touched" = "0" ]; then
        stage "  $label: nothing to remove"
    fi
}

do_cleanup_all() {
    # Run cleanup_old_version for v4.1, v4, v2 in order.
    # Respects DRY_RUN. Used by both install.sh (when MODE=clean) and
    # uninstall_others.sh (standalone wrapper).
    local header_verb="removing"
    [ "${DRY_RUN:-0}" = "1" ] && header_verb="would remove"
    stage "device cleanup: $header_verb every non-v5 install (v2 / v4 / v4.1)"
    stage "  v4.1 ..."
    cleanup_old_version "v4.1" \
        "custom_sortingv4_1" "tune_uiv4_1" "custom_sorting_nodev4.1.launch.py" \
        "jetarm_v4_1" "jetarm-sort-v4.1" "jetarm-v4.1" \
        ".jetarm_v4_1.env" "jetarm_v4_1_src"
    stage "  v4 ..."
    cleanup_old_version "v4" \
        "custom_sortingv4" "tune_uiv4" "custom_sorting_nodev4.launch.py" \
        "jetarm_v4" "jetarm-sort-v4" "jetarm-v4" \
        ".jetarm_v4.env" "jetarm_v4_src"
    stage "  v2 ..."
    cleanup_old_version "v2" \
        "custom_sortingv2" "tune_ui" "custom_sorting_nodev2.launch.py" \
        "jetarm" "jetarm-sort-v2" "jetarm-v2" \
        ".jetarm_v2.env" "jetarm_v2_src"

    # Shared resources: do NOT delete (v4 + v4.1 both used this dir).
    if [ -d "$HOME/jetarm_v4_profiles" ]; then
        stage "  [kept] $HOME/jetarm_v4_profiles (shared by v4 + v4.1; may hold your tuned YAMLs)"
    fi
}

detect_prior_versions() {
    # Returns 0 (true) if any non-v5 install artefact is present on disk.
    # Cheap probes only - matches what cleanup_old_version would remove.
    local d f e
    for d in "$HOME/jetarm_v4_1" "$HOME/jetarm_v4" "$HOME/jetarm"; do
        [ -d "$d" ] || continue
        # Only count it as an install if a launcher script is inside -
        # an empty dir left by an aborted run shouldn't trigger the prompt.
        if compgen -G "$d/launch_v*.sh" >/dev/null; then
            return 0
        fi
    done
    for f in "$HOME/.jetarm_v4_1.env" "$HOME/.jetarm_v4.env" "$HOME/.jetarm_v2.env"; do
        [ -f "$f" ] && return 0
    done
    if [ -f "$APP_PKG/setup.py" ]; then
        for e in "custom_sortingv4_1 = app." \
                 "custom_sortingv4 = app." \
                 "custom_sortingv2 = app."; do
            grep -qF "$e" "$APP_PKG/setup.py" && return 0
        done
    fi
    for f in "$HOME/Desktop/jetarm-sort-v4.1.desktop" \
             "$HOME/Desktop/jetarm-sort-v4.desktop" \
             "$HOME/Desktop/jetarm-sort-v2.desktop"; do
        [ -f "$f" ] && return 0
    done
    return 1
}

prompt_cleanup_mode() {
    # Print "keep" or "clean" on stdout. Tiered UI: zenity -> whiptail -> text.
    # All tiers default to "keep" on cancel / empty input (safer default).
    local choice=""
    if command -v zenity >/dev/null 2>&1; then
        choice=$(zenity --list --radiolist \
            --title="JetArm Sort v5 installer" \
            --width=560 --height=260 \
            --text="Prior JetArm versions detected on this device.\nWhat should the v5 installer do with them?" \
            --column="Pick" --column="Mode" --column="Description" \
            TRUE  keep  "Keep old versions (install v5 alongside)" \
            FALSE clean "Clean old versions (only v5 remains)" \
            2>/dev/null) || choice="keep"
    elif command -v whiptail >/dev/null 2>&1; then
        choice=$(whiptail --title "JetArm Sort v5 installer" --menu \
            "Prior JetArm versions detected.\nWhat should the v5 installer do with them?" \
            15 72 2 \
            "keep"  "Keep old versions (install v5 alongside)" \
            "clean" "Clean old versions (only v5 remains)" \
            3>&1 1>&2 2>&3) || choice="keep"
    else
        local ans=""
        printf "[install] Prior JetArm versions detected.\n" >&2
        printf "[install]   [K] Keep old versions (install v5 alongside) - default\n" >&2
        printf "[install]   [C] Clean old versions (only v5 remains)\n" >&2
        read -rp "[install] Choice [K/c]: " ans
        case "${ans,,}" in
            c|clean) choice="clean" ;;
            *)       choice="keep"  ;;
        esac
    fi
    [ "$choice" = "clean" ] || choice="keep"
    printf '%s' "$choice"
}

# --- 1c. Device cleanup: resolve mode + dispatch ------------------------

# Resolve MODE from (in priority order): CLI flags, INSTALL_MODE env,
# disk detection (skip if no prior versions), then interactive prompt
# (with TTY guard - unattended runs default to keep).
MODE=""
if [ "$KEEP_OTHERS" = "1" ];  then MODE=keep;  fi
if [ "$CLEAN_OTHERS" = "1" ]; then MODE=clean; fi
if [ -z "$MODE" ] && [ -n "${INSTALL_MODE:-}" ]; then
    case "$INSTALL_MODE" in
        keep|clean) MODE="$INSTALL_MODE" ;;
        *) err "ignoring INSTALL_MODE='$INSTALL_MODE' (expected 'keep' or 'clean')" ;;
    esac
fi

if ! detect_prior_versions; then
    stage "device cleanup: no prior JetArm versions detected, skipping"
    MODE=none
elif [ -z "$MODE" ]; then
    stage "device cleanup: prior JetArm versions detected. Preview of what would be cleaned:"
    DRY_RUN=1 do_cleanup_all
    if [ -t 0 ]; then
        MODE=$(prompt_cleanup_mode)
        stage "user chose: $MODE"
    else
        MODE=keep
        stage "no TTY and no --keep-others/--clean-others/INSTALL_MODE - defaulting to keep"
        stage "  to clean instead, re-run with:   bash install.sh --clean-others"
        stage "  or set:                          INSTALL_MODE=clean bash install.sh"
    fi
fi

case "$MODE" in
    clean)
        do_cleanup_all
        ok "device cleanup done"
        ;;
    keep)
        stage "device cleanup: keeping prior versions per user choice"
        stage "  v5 will install alongside; v2/v4/v4.1 shortcuts and launchers remain"
        stage "  to clean later:   bash $V4/uninstall_others.sh"
        ;;
    none)
        : # already logged the skip message above
        ;;
esac

# --- 2. Copy node + UI ---------------------------------------------------

# We *symlink* the python sources from the git checkout into the ROS2 app
# package, and we run colcon with --symlink-install. Net effect: a future
# `git pull` inside the checkout is immediately picked up by the running
# ros2 node. No copy, no rebuild for pure-Python edits.
#
# Only `setup.py` / `package.xml` / launch file changes still need a colcon
# rebuild - and we already symlink the launch file, so even those edits
# propagate the moment you re-run a `ros2 launch`.
link_file() {
    local src="$1" dst="$2"
    if [ ! -e "$src" ]; then
        err "source file missing: $src"; return 1
    fi
    # If dst already symlinks to the right place, skip.
    if [ -L "$dst" ] && [ "$(readlink -f "$dst")" = "$(readlink -f "$src")" ]; then
        stage "  already linked: $dst"
        return 0
    fi
    # Otherwise replace whatever's there with a symlink.
    rm -f "$dst"
    ln -s "$src" "$dst"
    # Verify the new symlink resolves to a real file. `readlink -e` follows
    # the entire chain and only succeeds if every link in it exists. Without
    # this check, a partial v5 checkout could leave a dangling symlink that
    # blows up colcon build later with a confusing ENOENT.
    if ! readlink -e "$dst" >/dev/null 2>&1; then
        err "linked target does not resolve: $dst -> $src (symlink chain broken)"
        return 1
    fi
    stage "  linked: $dst -> $src"
}

stage "symlinking v4 sources into $APP_PKG/app/"
link_file "$V4/custom_sortingv5.py" "$APP_PKG/app/custom_sortingv5.py"
link_file "$V4/tune_uiv5.py"        "$APP_PKG/app/tune_uiv5.py"

# --- 3. Symlink launch file ---------------------------------------------

stage "symlinking launch file into $APP_PKG/launch/"
mkdir -p "$APP_PKG/launch"
link_file "$V4/custom_sorting_nodev5.launch.py" \
          "$APP_PKG/launch/custom_sorting_nodev5.launch.py"

# --- 4. Patch setup.py (idempotent) --------------------------------------

SETUP_PY="$APP_PKG/setup.py"
if [ ! -f "$SETUP_PY" ]; then
    err "$SETUP_PY missing - cannot register entry points"
    exit 1
fi

patch_entry() {
    local entry="$1"
    if grep -qF "$entry" "$SETUP_PY"; then
        stage "  already present: $entry"
    else
        stage "  inserting: $entry"
        # Append the new entry right before the closing of console_scripts list.
        # We look for a line containing only '],' inside the console_scripts list.
        python3 - "$SETUP_PY" "$entry" <<'PY'
import re, sys
path, entry = sys.argv[1], sys.argv[2]
text = open(path).read()
m = re.search(r"(\s*'console_scripts'\s*:\s*\[)(.*?)(\n\s*\])", text, re.S)
if not m:
    sys.stderr.write("could not find console_scripts list - patch manually\n"); sys.exit(2)
new_inner = m.group(2).rstrip() + f"\n            '{entry}',"
out = text[:m.start()] + m.group(1) + new_inner + m.group(3) + text[m.end():]
open(path, 'w').write(out)
PY
    fi
}

stage "patching $SETUP_PY entry_points"
patch_entry "custom_sortingv5 = app.custom_sortingv5:main"
patch_entry "tune_uiv5 = app.tune_uiv5:main"

# --- 5. Profiles ---------------------------------------------------------

stage "seeding profiles in $PROFILES_DIR"
mkdir -p "$PROFILES_DIR"
# v5 ships default.yaml (boot fallback) + yolo.yaml (sample model config;
# the Model-tab SAVE overwrites it). The vendor AprilTag calibration node
# is pulled from package 'app' by the launch include - no extra install.
for p in default.yaml yolo.yaml; do
    src="$V4/profiles/$p"
    dst="$PROFILES_DIR/$p"
    if [ -f "$dst" ]; then
        stage "  keeping existing $dst"
    elif [ ! -f "$src" ]; then
        # Don't let a missing seed abort the whole install under set -e.
        stage "  WARNING: $src missing - skipped (node falls back to built-in defaults)"
    else
        install -m 644 "$src" "$dst"
        stage "  installed $dst"
    fi
done

# --- 6. Launcher ---------------------------------------------------------

stage "installing launcher in $LAUNCHER_DIR (as symlinks - git pull auto-updates)"
mkdir -p "$LAUNCHER_DIR"
# Symlink (not copy) so `git -C $SRC_DIR pull` immediately updates the
# launcher without re-running install.sh. Round 11 fix: previous installs
# used `install -m 755` which COPIED the file - so when the user pulled
# new launcher changes (e.g. the Zenity/text-mode profile modal), the
# desktop icon kept running the stale copy from initial install. The -f
# flag overwrites any existing copy or symlink.
ln -sf "$V4/launch_v5.sh"         "$LAUNCHER_DIR/launch_v5.sh"
ln -sf "$V4/image_view_chain.sh"    "$LAUNCHER_DIR/image_view_chain.sh"
ln -sf "$V4/re-enable-factory.sh"   "$LAUNCHER_DIR/re-enable-factory.sh"
ln -sf "$V4/match_launcher_env.sh"  "$LAUNCHER_DIR/match_launcher_env.sh"

# --- 6b. Best-effort install of an image viewer --------------------------
# image_view / rqt_image_view aren't always preinstalled on the Hiwonder
# container image. Try to install rqt_image_view via apt (it pulls in
# image_view as a dep). If apt isn't available or it fails we just
# warn and let the chain script's browser fallback handle it.
need_rqt=0; need_iv=0; need_wvs=0; need_zenity=0; need_whiptail=0
command -v rqt_image_view >/dev/null 2>&1 || need_rqt=1
# image_view is a ROS pkg, not a bin on PATH; check via ros2 pkg list
ros2 pkg list 2>/dev/null | grep -q '^image_view$' || need_iv=1
ros2 pkg list 2>/dev/null | grep -q '^web_video_server$' || need_wvs=1
# zenity: launcher modal's GUI tier. Falls back to whiptail/text without it.
command -v zenity >/dev/null 2>&1 || need_zenity=1
# whiptail: launcher modal's arrow-key TUI tier. Preinstalled on most
# Ubuntu/Debian but not guaranteed in minimal containers.
command -v whiptail >/dev/null 2>&1 || need_whiptail=1

if [ $((need_rqt + need_iv + need_wvs + need_zenity + need_whiptail)) -gt 0 ]; then
    pkgs=()
    [ $need_rqt      = 1 ] && pkgs+=("ros-${ROS_DISTRO:-humble}-rqt-image-view")
    [ $need_iv       = 1 ] && pkgs+=("ros-${ROS_DISTRO:-humble}-image-view")
    [ $need_wvs      = 1 ] && pkgs+=("ros-${ROS_DISTRO:-humble}-web-video-server")
    [ $need_zenity   = 1 ] && pkgs+=("zenity")
    [ $need_whiptail = 1 ] && pkgs+=("whiptail")
    stage "missing viewer packages: ${pkgs[*]}"
    if command -v apt-get >/dev/null 2>&1; then
        stage "running interactive 'sudo apt-get install' (may prompt for password)"
        if sudo apt-get install -y --no-install-recommends "${pkgs[@]}"; then
            ok "viewer packages installed"
        else
            err "apt install failed. Install manually:"
            err "  sudo apt install -y ${pkgs[*]}"
        fi
    else
        err "no apt-get on PATH - install manually:"
        err "  sudo apt install -y ${pkgs[*]}"
    fi
fi

# --- 7. Desktop shortcut -------------------------------------------------

stage "installing desktop shortcut"
mkdir -p "$HOME/.local/share/applications" "$HOME/Desktop"
DESKTOP_FILE="$HOME/Desktop/jetarm-sort-v5.desktop"
APP_FILE="$HOME/.local/share/applications/jetarm-sort-v5.desktop"

# Pick a terminal that will load ~/.bashrc (which is what prints the
# Hiwonder banner / sets CAMERA_TYPE etc.). gnome-terminal -- bash -i -c
# runs an INTERACTIVE bash so .bashrc IS sourced - which a bare
# `Terminal=true` + non-interactive `bash -c` does not.
#
# After the launcher exits we drop into an interactive shell so the user
# can read the log / re-run things.
if command -v gnome-terminal >/dev/null 2>&1; then
    EXEC_LINE="gnome-terminal --title='JetArm Sort v5' -- bash -i -c '\"$LAUNCHER_DIR/launch_v5.sh\"; exec bash -i'"
    USE_TERMINAL_FIELD="false"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
    EXEC_LINE="x-terminal-emulator -e bash -i -c '\"$LAUNCHER_DIR/launch_v5.sh\"; exec bash -i'"
    USE_TERMINAL_FIELD="false"
else
    # Fallback - rely on the desktop to open a terminal for us.
    EXEC_LINE="bash -i -c '\"$LAUNCHER_DIR/launch_v5.sh\"; exec bash -i'"
    USE_TERMINAL_FIELD="true"
fi

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=JetArm Sort v5
Comment=Launch the v5 custom sorting stack with hot-swap models and live tuner
Exec=$EXEC_LINE
Terminal=$USE_TERMINAL_FIELD
Icon=utilities-terminal
Categories=Development;Robotics;
StartupNotify=true
EOF
chmod +x "$DESKTOP_FILE"
cp "$DESKTOP_FILE" "$APP_FILE"
chmod +x "$APP_FILE"

# --- 8. Build -----------------------------------------------------------

if [ "$DO_BUILD" = "1" ]; then
    # The device cleanup stage removes v2/v4/v4.1 src-side launch symlinks
    # but cannot reach the matching install-side symlinks under
    # $WS_DIR/install/app/share/app/launch/. Those would survive as dangling
    # links and trip setuptools' develop step with a confusing ENOENT. Nuke
    # the app package's build + install trees so colcon rebuilds them
    # cleanly. Per-package only - other Hiwonder packages are untouched, and
    # the rebuild cost for `app` alone is 3-5 seconds.
    stage "wiping stale colcon artifacts for app (forces clean rebuild)"
    rm -rf "$WS_DIR/build/app" "$WS_DIR/install/app"
    stage "building app package with --symlink-install"
    # --symlink-install makes the install/ tree contain symlinks back into
    # src/, so Python source edits don't need a rebuild. Combined with the
    # source-file symlinks above, a `git pull` in $SRC_DIR is picked up by
    # the next `ros2 launch` with no colcon step. Only changes that need
    # entry_points re-registration (setup.py) need a rebuild.
    (cd "$WS_DIR" && colcon build --packages-select app --symlink-install) || {
        err "colcon build failed - see output above"
        exit 1
    }
    ok "colcon build done (symlinked)"
else
    stage "skipping colcon build (--no-build)"
fi

# --- 9. Sudoers (optional) ----------------------------------------------

if [ "$DO_SUDOERS" = "1" ]; then
    stage "installing NOPASSWD sudoers rule for systemctl stop/disable/restart"
    SUDOERS_FILE="/etc/sudoers.d/jetarm-v5"
    USER_NAME="$(whoami)"
    # v5 needs stop+disable every launch (factory service kept off);
    # restart is also allowed for the optional FORCE_SERVICE_RESTART path.
    RULES=$(cat <<EOF
$USER_NAME ALL=(ALL) NOPASSWD: /bin/systemctl stop start_app_node.service
$USER_NAME ALL=(ALL) NOPASSWD: /bin/systemctl disable start_app_node.service
$USER_NAME ALL=(ALL) NOPASSWD: /bin/systemctl restart start_app_node.service
$USER_NAME ALL=(ALL) NOPASSWD: /bin/systemctl start start_app_node.service
EOF
)
    TMP=$(mktemp)
    printf '%s\n' "$RULES" > "$TMP"
    if visudo -cf "$TMP" >/dev/null 2>&1; then
        sudo install -m 440 "$TMP" "$SUDOERS_FILE"
        rm -f "$TMP"
        ok "sudoers rules installed at $SUDOERS_FILE"
    else
        err "visudo rejected the generated rule - skipping"
        rm -f "$TMP"
    fi
fi

# --- 10. Done -----------------------------------------------------------

cat <<EOF

================================================================
$(ok 'INSTALL COMPLETE')
================================================================
Symlinks installed (no rebuild needed for Python edits):

  $APP_PKG/app/custom_sortingv5.py
    -> $V4/custom_sortingv5.py
  $APP_PKG/app/tune_uiv5.py
    -> $V4/tune_uiv5.py
  $APP_PKG/launch/custom_sorting_nodev5.launch.py
    -> $V4/custom_sorting_nodev5.launch.py

Workspace built with --symlink-install, so install/ also points back at
src/. Net effect: when you want to pull updates, just do

  git -C $SRC_DIR pull

and re-launch. No colcon, no copy. Only re-run install.sh if setup.py
or package.xml changed upstream (entry_points / dependencies).

Other files installed:

  $PROFILES_DIR/{default,yolo}.yaml
  $LAUNCHER_DIR/launch_v5.sh
  $DESKTOP_FILE
  $APP_FILE

To launch:

  # one-click: double-click the "JetArm Sort v5" icon on your desktop
  # (right-click -> Allow Launching the first time on GNOME)
  #
  # or from a terminal:
  $LAUNCHER_DIR/launch_v5.sh

  # with debug stage prints:
  JETARM_V5_DEBUG=1 $LAUNCHER_DIR/launch_v5.sh

  # boot into a preset profile:
  $LAUNCHER_DIR/launch_v5.sh

If the camera doesn't come up, see:
  custom_sortingv5/QUICK_SETUP_HIWONDER.md  in $SRC_DIR

================================================================
EOF
