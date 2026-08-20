#!/usr/bin/env bash
# Stop every tog-sim / Gazebo / ROS process left over from a previous run (stale bridges publish extra /clock!).
for pat in "ign gazebo" "ruby /usr/bin/ign" parameter_bridge image_bridge product_spawner motion_server vacuum_bridge \
           demo_pick_gt robot_state_publisher static_transform_publisher "ros2 launch" "controller_manager/spawner"; do
  pkill -f -- "$pat" 2>/dev/null || true
done
sleep 1
left=$(pgrep -af "ign gazebo|parameter_bridge|image_bridge|product_spawner|motion_server|vacuum_bridge|robot_state_publisher|static_transform_publisher|ros2 launch" | grep -v killall || true)
if [ -n "$left" ]; then echo "still running:"; echo "$left"; pkill -9 -f "ign gazebo" 2>/dev/null || true; else echo "clean"; fi
