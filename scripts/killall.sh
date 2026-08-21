#!/usr/bin/env bash
# Stop every tog-sim process left over from a previous run (stale bridges publish extra /clock!).
# Never kills the shell that invoked it (its command line may contain the same words), and never touches ROS/Gazebo
# processes of other projects: a candidate must mention "togsim" on its command line or be an Ignition/Fortress server.
me=$$; parent=$PPID
ours() {  # $1 = pid
  local cmd; cmd=$(tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null) || return 1
  case "$cmd" in *togsim*|*"ign gazebo"*|*"ruby /usr/bin/ign"*) return 0 ;; esac
  return 1
}
kill_pat() {
  for pid in $(pgrep -f -- "$1"); do
    [ "$pid" = "$me" ] || [ "$pid" = "$parent" ] || ! ours "$pid" || kill "$pid" 2>/dev/null || true
  done
}
for pat in "ign gazebo" "ruby /usr/bin/ign" parameter_bridge image_bridge product_spawner motion_server vacuum_bridge run_datagen run_cycle \
           demo_pick_gt robot_state_publisher static_transform_publisher "ros2 launch" "controller_manager/spawner"; do
  kill_pat "$pat"
done
sleep 1
left=""
for pat in "ign gazebo" parameter_bridge image_bridge product_spawner motion_server vacuum_bridge robot_state_publisher "ros2 launch"; do
  for pid in $(pgrep -f -- "$pat"); do
    [ "$pid" = "$me" ] || [ "$pid" = "$parent" ] || ! ours "$pid" || left="$left $pid"
  done
done
if [ -n "$left" ]; then kill -9 $left 2>/dev/null || true; echo "force-killed:$left"; else echo "clean"; fi
