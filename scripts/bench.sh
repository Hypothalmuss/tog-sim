#!/usr/bin/env bash
# Repeatable benchmark: N runs of scripts/m4_validate.sh with fixed seeds, one JSON summary per run + a table.
#   scripts/bench.sh <name> [repeats=2] [cycles=20] [perception=vision] [belt=0.10]
#   env passthrough: M4_RATE M4_CLASSES M4_PROFILE M4_WARMUP (seed = 100+i)
set +u
name=${1:?name}; repeats=${2:-2}; cycles=${3:-20}; mode=${4:-vision}; speed=${5:-0.10}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out=$HOME/togsim_data/bench; mkdir -p "$out"
for i in $(seq 1 "$repeats"); do
  run="$out/${name}_$i"; rm -rf "$run"
  M4_SEED=$((100 + i)) M4_OUT="$run" bash "$here/scripts/m4_validate.sh" "$cycles" "$mode" "$speed" >"$out/${name}_$i.out" 2>&1
  python3 "$here/scripts/bench_summary.py" "$run/run_cycle.log" "$run/sim1.log" >"$out/${name}_$i.json"
  echo "[bench] $name run $i: $(python3 -c "import json; d=json.load(open('$out/${name}_$i.json')); print(f\"{d['cycles']}/{d['attempts']} cycles, {d['cpm']:.1f} cpm, motion {d['motion_mean_s']:.2f} s, placement {d['placement_mm_mean']} mm\")")"
done
