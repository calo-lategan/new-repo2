#!/usr/bin/env bash
# Source this in any second terminal where you want to probe the running
# JetArm v4.1 stack. Matches the launcher's domain + ROS env exactly.
#
#   source ~/jetarm_v4_1/match_launcher_env.sh
#
# Then:
#   ros2 node list
#   ros2 topic hz /custom_sortingv4_1/image_result
#   rqt_image_view /custom_sortingv4_1/image_result
#
# If you launched with a custom domain (e.g. ROS_DOMAIN_ID=5 ~/jetarm_v4_1/launch_v4.1.sh),
# set the same value here BEFORE sourcing this file.

# Pick up the same persistent overrides the launcher does.
if [ -f "$HOME/.jetarm_v4_1.env" ]; then
    # shellcheck disable=SC1091
    . "$HOME/.jetarm_v4_1.env"
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
echo "    rqt_image_view on PATH: $(command -v rqt_image_view || echo NO)"
echo ""
echo "    ros2 node list:"
ros2 node list 2>/dev/null | sed 's/^/      /' || echo "      (no daemon yet; try again in a second)"
