#!/usr/bin/env bash
# Regenerate docs/benchmarks.md from the recorded benches (scripts/bench.sh results in ~/togsim_data/bench).
# The rows and their labels are the documented history of the throughput/precision work; add a line per new bench.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
python3 scripts/bench_report.py \
  ref_cart p1_out05 p1_pitch045 p1_max p1_overlap p3_cart_v2 p3_mix_v2 \
  --label "ref_cart=cartons 60/min, fast, outfeed 0.06 m/s (baseline 181ebb9)" \
  --label "p1_out05=P1a: outfeed 0.05 m/s (rejected: fewer trays per minute)" \
  --label "p1_pitch045=P1b: tray pitch 0.45 m (rejected after 1 run: no-seal picks, idle up)" \
  --label "p1_max=P1d: max motion profile (not adopted: 3x jerk, cpm within CI)" \
  --label "p1_overlap=P1c: leave at unseal, before the place dwell ends (rejected: precision p95 8.0 mm, cpm down)" \
  --label "p3_cart_v2=P3: seg_v2 weights (bar trays in the training data), cartons 60/min" \
  --label "p3_mix_v2=P3: seg_v2 weights, cartons + bars 30/min on tray_2x4 + tray_bar_2x3" \
  "$@"
