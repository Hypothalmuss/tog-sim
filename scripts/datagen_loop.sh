#!/usr/bin/env bash
# Generate the synthetic segmentation dataset robustly: (re)launch the headless cell with panoptic cameras, run the
# resumable run_datagen, and restart the simulator whenever it stalls (run_datagen exits 3) until all scenes exist.
#   scripts/datagen_loop.sh [frames=400] [out_dir=~/togsim_data/seg_v1] [max_restarts=20]
set +u  # ROS setup.bash references unset variables
frames=${1:-400}; out=${2:-$HOME/togsim_data/seg_v1}; max=${3:-20}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$here/scripts/env.sh"
log=${DATAGEN_LOG:-/tmp/togsim_datagen_$(date +%H%M%S).log}
for attempt in $(seq 1 "$max"); do
  "$here/scripts/killall.sh" >/dev/null
  echo "=== attempt $attempt: launching sim ($(date +%T))" | tee -a "$log"
  setsid ros2 launch togsim_bringup sim_full.launch.py gui:=false products:=false segmentation:=true >"${log%.log}_sim$attempt.log" 2>&1 </dev/null &
  ready=0
  for i in $(seq 1 90); do
    if timeout 3 ros2 topic echo --once /cam_pick/segmentation/labels_map --field header >/dev/null 2>&1 \
       && timeout 3 ros2 topic echo --once /cam_place/image --field header >/dev/null 2>&1; then ready=1; break; fi
    sleep 2
  done
  if [ $ready = 0 ]; then echo "sim did not come up, retrying" | tee -a "$log"; continue; fi
  sleep 5
  ros2 run togsim_perception run_datagen --ros-args -p frames:="$frames" -p out_dir:="$out" -p seed:="$((1000 + attempt))" >>"$log" 2>&1
  rc=$?
  echo "=== run_datagen exit $rc ($(date +%T))" | tee -a "$log"
  "$here/scripts/killall.sh" >/dev/null
  if [ $rc = 0 ] || tail -n 20 "$log" | grep -q "done: .* scenes in"; then echo "dataset complete: $out" | tee -a "$log"; exit 0; fi
  sleep 3
done
echo "gave up after $max attempts" | tee -a "$log"; exit 1
