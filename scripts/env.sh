#!/usr/bin/env bash
# Source this in every shell that talks to tog-sim:  source ~/tog-sim/scripts/env.sh
# - own ROS domain, so other ROS 2 stacks on this machine/LAN (e.g. a second Gazebo publishing /clock) never mix with ours
# - workspace overlay, GPU rendering for Gazebo sensors, optional GPU venv for YOLO (torch built for the installed driver)
export ROS_DOMAIN_ID="${TOGSIM_ROS_DOMAIN_ID:-42}"
export LIBGL_ALWAYS_SOFTWARE=0
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_here/install/setup.bash" ] && source "$_here/install/setup.bash"
[ -f "$HOME/togsim_data/venv/bin/activate" ] && source "$HOME/togsim_data/venv/bin/activate"
unset _here
