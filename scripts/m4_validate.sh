#!/usr/bin/env bash
# M4 validation: continuous picking from moving belts with conveyor_tracker + TRACK_CART segments.
#   scripts/m4_validate.sh [cycles=12] [perception=gt|vision] [belt_speed=0.10]   (env: M4_RATE products/min as a float e.g. 40.0, M4_WARMUP, M4_TIMEOUT)
# Results in $M4_OUT (default ~/togsim_data/m4_<timestamp>): sim.log tracker.log [perception.log] run_cycle.log
set +u
cycles=${1:-12}; mode=${2:-gt}; speed=${3:-0.10}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$here/scripts/env.sh"
out=${M4_OUT:-$HOME/togsim_data/m4_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$out"
wait_for() { local t=$1; shift; for _ in $(seq 1 "$t"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done; return 1; }
cleanup() { "$here/scripts/killall.sh" >/dev/null; }
trap cleanup EXIT
cleanup
echo "[m4] launching cell (headless, products on) -> $out"
launch_cell() {  # Fortress occasionally hangs at robot spawn (controller_manager never appears): relaunch once
  for attempt in 1 2; do
    cleanup
    setsid ros2 launch togsim_bringup sim_full.launch.py gui:=false infeed_speed:=$speed outfeed_speed:=$speed product_rate:=${M4_RATE:-24.0} >"$out/sim$attempt.log" 2>&1 </dev/null &
    wait_for 150 bash -c "ros2 action list | grep -q /togsim/execute_motion" || continue
    wait_for 60 timeout 3 ros2 topic echo --once /joint_states --field header && return 0
    echo "[m4] simulator hung at start-up (no joint states), relaunching"
  done
  return 1
}
launch_cell || { echo "[m4] cell did not come up"; exit 1; }
if [ "$mode" = vision ]; then
  wait_for 60 timeout 3 ros2 topic echo --once /cam_pick/depth_image --field header || { echo "[m4] cameras missing"; exit 1; }
  setsid ros2 launch togsim_perception perception.launch.py >"$out/perception.log" 2>&1 </dev/null &
  wait_for 90 timeout 3 ros2 topic echo --once /togsim/pick_candidates --field frame_seq || { echo "[m4] no pick candidates"; exit 1; }
fi
echo "[m4] conveyor_tracker source=$mode"
setsid ros2 run togsim_perception conveyor_tracker --ros-args -p use_sim_time:=true -p source:=$mode >"$out/tracker.log" 2>&1 </dev/null &
wait_for 60 timeout 3 ros2 topic echo --once /togsim/tracks/products --field frame_seq || { echo "[m4] no tracks"; exit 1; }
sleep ${M4_WARMUP:-20}
echo "[m4] run_cycle continuous perception:=$mode cycles:=$cycles belt $speed m/s"
timeout ${M4_TIMEOUT:-900} ros2 run togsim_task run_cycle --ros-args -p perception:=$mode -p continuous:=true -p cycles:="$cycles" -p belt_speed:=$speed -p use_sim_time:=true >"$out/run_cycle.log" 2>&1
grep -E "cycle [0-9]+:|finished|no seal|failed|jam" "$out/run_cycle.log" | tail -8
echo "[m4] done -> $out"
