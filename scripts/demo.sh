#!/usr/bin/env bash
# Visible demo of the whole stack: Gazebo GUI cell with products, YOLO-seg perception, conveyor tracker and continuous
# vision-driven pick & place from the moving belts, plus a window with the live segmentation overlay.
#   scripts/demo.sh [perception=vision|gt] [cycles=1000] [belt_speed=0.10]      stop with scripts/killall.sh
set +u
mode=${1:-vision}; cycles=${2:-1000}; speed=${3:-0.10}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$here/scripts/env.sh"
export DISPLAY=${DISPLAY:-:1}
log=${DEMO_LOG:-/tmp/togsim_demo}; mkdir -p "$log"
wait_for() { local t=$1; shift; for _ in $(seq 1 "$t"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done; return 1; }
"$here/scripts/killall.sh" >/dev/null
echo "[demo] Gazebo GUI cell (products ${DEMO_RATE:-30.0}/min, belts $speed m/s) -> logs in $log"
launch_cell() {  # Fortress occasionally hangs at robot spawn (no joint states): relaunch once
  for attempt in 1 2; do
    [ "$attempt" = 2 ] && { echo "[demo] simulator hung at start-up, relaunching"; "$here/scripts/killall.sh" >/dev/null; sleep 2; }
    setsid ros2 launch togsim_bringup sim_full.launch.py gui:=true infeed_speed:=$speed outfeed_speed:=${DEMO_OUTFEED:-$speed} product_rate:=${DEMO_RATE:-30.0} product_classes:=${DEMO_CLASSES:-product_bar,product_carton} ${DEMO_TRAYS:+tray_models:=$DEMO_TRAYS} motion_profile:=${DEMO_PROFILE:-smooth} >"$log/sim.log" 2>&1 </dev/null &
    wait_for 180 bash -c "ros2 action list | grep -q /togsim/execute_motion" || continue
    wait_for 90 timeout 3 ros2 topic echo --once /joint_states --field header && return 0
  done
  return 1
}
launch_cell || { echo "[demo] simulator hung at start-up twice: run scripts/killall.sh and try again"; exit 1; }
if [ "$mode" = vision ]; then
  wait_for 60 timeout 3 ros2 topic echo --once /cam_pick/depth_image --field header || { echo "[demo] cameras missing"; exit 1; }
  echo "[demo] perception (YOLO11n-seg on the GPU, pick poses, tray vacancy)"
  setsid ros2 launch togsim_perception perception.launch.py ${DEMO_TRAYS:+tray_models:=$DEMO_TRAYS} >"$log/perception.log" 2>&1 </dev/null &
  wait_for 120 timeout 3 ros2 topic echo --once /togsim/pick_candidates --field frame_seq || { echo "[demo] no pick candidates"; exit 1; }
  echo "[demo] live segmentation overlay window (rqt_image_view /togsim/debug/cam_pick)"
  setsid ros2 run rqt_image_view rqt_image_view /togsim/debug/cam_pick >"$log/rqt.log" 2>&1 </dev/null &
fi
echo "[demo] conveyor_tracker source=$mode"
setsid ros2 run togsim_perception conveyor_tracker --ros-args -p use_sim_time:=true -p source:=$mode >"$log/tracker.log" 2>&1 </dev/null &
wait_for 60 timeout 3 ros2 topic echo --once /togsim/tracks/products --field frame_seq || { echo "[demo] no tracks"; exit 1; }
echo "[demo] operator HMI at http://localhost:${DEMO_HMI_PORT:-8080} (status, belts, start/stop, tray occupancy)"
setsid ros2 run togsim_hmi hmi_server --ros-args -p use_sim_time:=true -p port:=${DEMO_HMI_PORT:-8080} >"$log/hmi.log" 2>&1 </dev/null &
sleep 10
echo "[demo] continuous pick & place (perception:=$mode, $cycles cycles) - stop everything with scripts/killall.sh"
setsid ros2 run togsim_task run_cycle --ros-args -p perception:=$mode -p continuous:=true -p cycles:=$cycles -p belt_speed:=$speed -p outfeed_speed:=${DEMO_OUTFEED:-0.0} -p use_sim_time:=true >"$log/run_cycle.log" 2>&1 </dev/null &
echo "[demo] running; progress: tail -f $log/run_cycle.log"
