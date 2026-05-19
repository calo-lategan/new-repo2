#!/usr/bin/env bash
# JetArm Sort v4 - one-shot installer for the Hiwonder container.
#
# What it does (idempotent - safe to re-run):
#   1. Clones/refreshes this repo into ~/jetarm_v4_src
#   2. Copies the v4 python files into ~/ros2_ws/src/app/app/
#   3. Copies the v4 launch file into ~/ros2_ws/src/app/launch/
#   4. Adds the v4 console_scripts entries to ~/ros2_ws/src/app/setup.py
#      (only if they're not already there)
#   5. Seeds ~/jetarm_v4_profiles/ with default.yaml, fast.yaml, precision.yaml
#   6. Installs ~/jetarm_v4/launch_v4.sh
#   7. Installs the desktop shortcut to ~/Desktop and ~/.local/share/applications
#   8. Runs `colcon build --packages-select app`
#   9. (optional, with --sudoers) writes a NOPASSWD rule for the systemctl
#      restart so the desktop shortcut is truly one-click.
#  10. Prints clear next-step instructions.
#
# This is designed to be run inside the Hiwonder container. From the host
# you can either ssh / lxc-attach into the container and run the curl one-
# liner from INSTALL.md, or run this script from a checkout you already
# have.
#
# Usage:
#   ./install.sh                  # default install
#   ./install.sh --sudoers        # also install the NOPASSWD sudoers rule
#   ./install.sh --no-build       # skip colcon build (do it yourself)
#   REPO=...  BRANCH=main  ./install.sh
#
# Env overrides:
#   REPO=https://github.com/calo-lategan/new-repo2.git
#   BRANCH=main
#   SRC_DIR=$HOME/jetarm_v4_src
#   WS_DIR=$HOME/ros2_ws
#   PROFILES_DIR=$HOME/jetarm_v4_profiles
#   LAUNCHER_DIR=$HOME/jetarm_v4

set -e

REPO="${REPO:-https://github.com/calo-lategan/new-repo2.git}"
BRANCH="${BRANCH:-main}"
SRC_DIR="${SRC_DIR:-$HOME/jetarm_v4_src}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
PROFILES_DIR="${PROFILES_DIR:-$HOME/jetarm_v4_profiles}"
LAUNCHER_DIR="${LAUNCHER_DIR:-$HOME/jetarm_v4}"
APP_PKG="${APP_PKG:-$WS_DIR/src/app}"
DO_SUDOERS=0
DO_BUILD=1

for arg in "$@"; do
    case "$arg" in
        --sudoers)  DO_SUDOERS=1 ;;
        --no-build) DO_BUILD=0 ;;
        --help|-h)
            sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "unknown arg: $arg"; exit 1 ;;
    esac
done

stage() { printf "\033[1;36m[install]\033[0m %s\n" "$*"; }
ok()    { printf "\033[1;32m[install]\033[0m %s\n" "$*"; }
err()   { printf "\033[1;31m[install]\033[0m %s\n" "$*" >&2; }

# --- 0. Sanity ------------------------------------------------------------

stage "JetArm Sort v4 installer"
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
V4="$SRC_DIR/v4 custom sorting"
if [ ! -d "$V4" ]; then
    err "v4 folder not found inside the checkout: $V4"
    exit 1
fi
ok "source ready"

# --- 2. Copy node + UI ---------------------------------------------------

stage "copying v4 sources into $APP_PKG/app/"
install -m 644 "$V4/custom_sortingv4.py" "$APP_PKG/app/custom_sortingv4.py"
install -m 644 "$V4/tune_uiv4.py"        "$APP_PKG/app/tune_uiv4.py"

# --- 3. Copy launch file -------------------------------------------------

stage "copying launch file into $APP_PKG/launch/"
mkdir -p "$APP_PKG/launch"
install -m 644 "$V4/custom_sorting_nodev4.launch.py" "$APP_PKG/launch/custom_sorting_nodev4.launch.py"

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
patch_entry "custom_sortingv4 = app.custom_sortingv4:main"
patch_entry "tune_uiv4 = app.tune_uiv4:main"

# --- 5. Profiles ---------------------------------------------------------

stage "seeding profiles in $PROFILES_DIR"
mkdir -p "$PROFILES_DIR"
for p in default.yaml fast.yaml precision.yaml; do
    src="$V4/profiles/$p"
    dst="$PROFILES_DIR/$p"
    if [ -f "$dst" ]; then
        stage "  keeping existing $dst"
    else
        install -m 644 "$src" "$dst"
        stage "  installed $dst"
    fi
done

# --- 6. Launcher ---------------------------------------------------------

stage "installing launcher in $LAUNCHER_DIR"
mkdir -p "$LAUNCHER_DIR"
install -m 755 "$V4/launch_v4.sh" "$LAUNCHER_DIR/launch_v4.sh"

# --- 7. Desktop shortcut -------------------------------------------------

stage "installing desktop shortcut"
mkdir -p "$HOME/.local/share/applications" "$HOME/Desktop"
DESKTOP_FILE="$HOME/Desktop/jetarm-sort-v4.desktop"
APP_FILE="$HOME/.local/share/applications/jetarm-sort-v4.desktop"
cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=JetArm Sort v4
Comment=Launch the v4 custom sorting stack with hot-swap models and live tuner
Exec=bash -c '"$LAUNCHER_DIR/launch_v4.sh"; exec bash'
Terminal=true
Icon=utilities-terminal
Categories=Development;Robotics;
StartupNotify=true
EOF
chmod +x "$DESKTOP_FILE"
cp "$DESKTOP_FILE" "$APP_FILE"
chmod +x "$APP_FILE"

# --- 8. Build -----------------------------------------------------------

if [ "$DO_BUILD" = "1" ]; then
    stage "building app package (colcon)"
    (cd "$WS_DIR" && colcon build --packages-select app) || {
        err "colcon build failed - see output above"
        exit 1
    }
    ok "colcon build done"
else
    stage "skipping colcon build (--no-build)"
fi

# --- 9. Sudoers (optional) ----------------------------------------------

if [ "$DO_SUDOERS" = "1" ]; then
    stage "installing NOPASSWD sudoers rule for systemctl restart"
    SUDOERS_FILE="/etc/sudoers.d/jetarm-v4"
    USER_NAME="$(whoami)"
    RULE="$USER_NAME ALL=(ALL) NOPASSWD: /bin/systemctl restart start_app_node.service"
    TMP=$(mktemp)
    printf '%s\n' "$RULE" > "$TMP"
    if visudo -cf "$TMP" >/dev/null 2>&1; then
        sudo install -m 440 "$TMP" "$SUDOERS_FILE"
        rm -f "$TMP"
        ok "sudoers rule installed at $SUDOERS_FILE"
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
What was installed:

  $APP_PKG/app/custom_sortingv4.py
  $APP_PKG/app/tune_uiv4.py
  $APP_PKG/launch/custom_sorting_nodev4.launch.py
  $PROFILES_DIR/{default,fast,precision}.yaml
  $LAUNCHER_DIR/launch_v4.sh
  $DESKTOP_FILE
  $APP_FILE

To launch:

  # one-click: double-click the "JetArm Sort v4" icon on your desktop
  # (right-click -> Allow Launching the first time on GNOME)
  #
  # or from a terminal:
  $LAUNCHER_DIR/launch_v4.sh

  # with debug stage prints:
  JETARM_V4_DEBUG=1 $LAUNCHER_DIR/launch_v4.sh

  # boot into a preset profile:
  $LAUNCHER_DIR/launch_v4.sh profile:=fast

If the camera doesn't come up, see:
  v4 custom sorting/QUICK_SETUP_HIWONDER.md  in $SRC_DIR

================================================================
EOF
