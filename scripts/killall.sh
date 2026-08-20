#!/usr/bin/env bash
# Stop every tog-sim / Gazebo / ROS process left over from a previous run (stale bridges publish extra /clock!).
# Never kills the shell that invoked it (its command line may contain the same words).
me=$$; parent=$PPID
kill_pat() {
  for pid in $(pgrep -f -- "$1"); do
    [ "$pid" = "$me" ] || [ "$pid" = "$parent" ] || kill "$pid" 2>/dev/null || true
  done
}
for pat in "ign gazebo" "ruby /usr/bin/ign" parameter_bridge image_bridge product_spawner motion_server vacuum_bridge \
           demo_pick_gt robot_state_publisher static_transform_publisher "ros2 launch" "controller_manager/spawner"; do
  kill_pat "$pat"
done
sleep 1
left=""
for pat in "ign gazebo" parameter_bridge image_bridge product_spawner motion_server vacuum_bridge robot_state_publisher "ros2 launch"; do
  for pid in $(pgrep -f -- "$pat"); do
    [ "$pid" = "$me" ] || [ "$pid" = "$parent" ] || left="$left $pid"
  done
done
if [ -n "$left" ]; then kill -9 $left 2>/dev/null || true; echo "force-killed:$left"; else echo "clean"; fi
