#!/usr/bin/env bash
# M3 validation: populated cell (headless) -> perception stack -> vision-vs-GT metrics -> vision-driven pick-and-place.
#   scripts/m3_validate.sh [cycles=12] [eval_frames=150] [weights=~/togsim_data/weights/togsim_seg.pt]
# Results land in $M3_OUT (default ~/togsim_data/m3_<timestamp>): sim.log perception.log eval_pick_poses.json run_cycle.log
set +u
cycles=${1:-12}; frames=${2:-150}; weights=${3:-$HOME/togsim_data/weights/togsim_seg.pt}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$here/scripts/env.sh"
out=${M3_OUT:-$HOME/togsim_data/m3_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$out"
wait_for() {  # wait_for <seconds> <command...>
  local t=$1; shift
  for _ in $(seq 1 "$t"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done
  return 1
}
cleanup() { "$here/scripts/killall.sh" >/dev/null; }
trap cleanup EXIT
cleanup
echo "[m3] launching cell (products on, headless) -> $out"
launch_cell() {  # Fortress occasionally hangs at robot spawn (controller_manager never appears): relaunch once
  for attempt in 1 2; do
    cleanup
    setsid ros2 launch togsim_bringup sim_full.launch.py gui:=false segmentation:=${M3_SEGMENTATION:-false} >"$out/sim$attempt.log" 2>&1 </dev/null &
    wait_for 150 bash -c "ros2 action list | grep -q /togsim/execute_motion" || continue
    wait_for 60 timeout 3 ros2 topic echo --once /joint_states --field header && return 0
    echo "[m3] simulator hung at start-up (no joint states), relaunching"
  done
  return 1
}
launch_cell || { echo "[m3] cell did not come up"; exit 1; }
wait_for 60 timeout 3 ros2 topic echo --once /cam_pick/depth_image --field header || { echo "[m3] cameras missing"; exit 1; }
echo "[m3] launching perception (weights: $weights)"
setsid ros2 launch togsim_perception perception.launch.py weights:="$weights" >"$out/perception.log" 2>&1 </dev/null &
wait_for 90 timeout 3 ros2 topic echo --once /togsim/pick_candidates --field frame_seq || { echo "[m3] no pick candidates"; exit 1; }
echo "[m3] waiting for products to reach the pick camera"
sleep ${M3_WARMUP:-45}  # products spawn at x=-1.05 and ride at 0.1 m/s: the first ones reach the camera after ~10-15 s
echo "[m3] eval_pick_poses ($frames frames)"
timeout 600 ros2 run togsim_perception eval_pick_poses --ros-args -p frames:="$frames" -p use_sim_time:=true -p out:="$out/eval_pick_poses.json" >"$out/eval.log" 2>&1
python3 - "$out/eval_pick_poses.json" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
print(f"[m3] recall {s['recall']:.2f} precision {s['precision']:.2f} class-acc {s['class_accuracy']:.2f} | "
      f"xy {s['xy_error_mm']['mean']:.1f} mm (p95 {s['xy_error_mm']['p95']:.1f}) | height {s['height_error_mm']['mean']:.1f} mm | "
      f"yaw {s['yaw_error_deg']['mean']:.1f} deg (p95 {s['yaw_error_deg']['p95']:.1f}) | {s['matched']}/{s['gt_products']} matched in {s['frames']} frames")
PY
echo "[m3] run_cycle perception:=vision cycles:=$cycles"
timeout 900 ros2 run togsim_task run_cycle --ros-args -p perception:=vision -p cycles:="$cycles" -p use_sim_time:=true >"$out/run_cycle.log" 2>&1
grep -E "cycle [0-9]+:|finished|no seal|jam|abort" "$out/run_cycle.log" | tail -6
echo "[m3] done -> $out"
