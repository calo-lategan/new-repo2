#!/usr/bin/env bash
# Source this in any second terminal where you want to probe the running
# JetArm v5 stack. Matches the launcher's domain + ROS env exactly.
#
#   source ~/jetarm_v5/match_launcher_env.sh
#
# Then:
#   ros2 node list
#   ros2 topic hz /custom_sortingv5/image_result
#   rqt_image_view /custom_sortingv5/image_result
#
# If you launched with a custom domain (e.g. ROS_DOMAIN_ID=5 ~/jetarm_v5/launch_v5.sh),
# set the same value here BEFORE sourcing this file.

# Pick up the same persistent overrides the launcher does.
if [ -f "$HOME/.jetarm_v5.env" ]; then
    # shellcheck disable=SC1091
    . "$HOME/.jetarm_v5.env"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Standard ROS env. Errors are silenced because some Hiwonder workspace
# setups print noise about stale workspace references that don't affect
# the package discovery.
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
source /home/ubuntu/third_party_ros2/orbbec_ws/install/setup.bash 2>/dev/null

echo "==> matched launcher env:"
echo "    ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "    RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
# rqt_image_view is an ament-python plugin, NOT a system binary.
# `command -v rqt_image_view` will say NO even when ros-humble-rqt-image-view
# is installed - that's a misleading signal. Detect via `ros2 pkg` instead.
if ros2 pkg executables rqt_image_view 2>/dev/null | grep -q '^rqt_image_view rqt_image_view$'; then
    echo "    rqt_image_view: installed (run via 'ros2 run rqt_image_view rqt_image_view <topic>')"
else
    echo "    rqt_image_view: NOT installed - sudo apt install ros-${ROS_DISTRO:-humble}-rqt-image-view"
fi
if ros2 pkg executables image_view 2>/dev/null | grep -q '^image_view image_view$'; then
    echo "    image_view:     installed (run via 'ros2 run image_view image_view --ros-args -r image:=<topic>')"
else
    echo "    image_view:     NOT installed - sudo apt install ros-${ROS_DISTRO:-humble}-image-view"
fi
echo ""
echo "    ros2 node list:"
ros2 node list 2>/dev/null | sed 's/^/      /' || echo "      (no daemon yet; try again in a second)"
