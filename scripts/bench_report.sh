#!/usr/bin/env bash
# Regenerate docs/benchmarks.md from the recorded benches (scripts/bench.sh results in ~/togsim_data/bench).
# The rows and their labels are the documented history of the throughput/precision work; add a line per new bench.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
python3 scripts/bench_report.py \
  ref_cart p1_out05 p1_pitch045 p1_max p1_overlap p3_cart_v2 p3_mix_v2 p3_cart_v3 p3_cart_v4 p3_mix_v5 p3_mix_v6 p5_dwell p5_dwell_lag \
  --label "ref_cart=cartons 60/min, fast, outfeed 0.06 m/s (baseline 181ebb9)" \
  --label "p1_out05=P1a: outfeed 0.05 m/s (rejected: fewer trays per minute)" \
  --label "p1_pitch045=P1b: tray pitch 0.45 m (rejected after 1 run: no-seal picks, idle up)" \
  --label "p1_max=P1d: max motion profile (not adopted: 3x jerk, cpm within CI)" \
  --label "p1_overlap=P1c: leave at unseal, before the place dwell ends (rejected: precision p95 8.0 mm, cpm down)" \
  --label "p3_cart_v2=P3: seg_v2 weights (bar trays in the training data), cartons 60/min" \
  --label "p3_mix_v2=P3: seg_v2 weights, cartons + bars 30/min on tray_2x4 + tray_bar_2x3 (before the bar fixes)" \
  --label "p3_cart_v3=P3: + tray-pose fix (on-tray visibility, per-axis lattice gates, products as lattice points) + grasp offset sum, cartons 60/min" \
  --label "p3_cart_v4=P3: + product lattice points only as fallback, box/rectangle product centre, cartons 60/min" \
  --label "p3_mix_v5=P3 final: tracked product centre, pocket 190x62, transfer 50 mm above walls; cartons + bars 30/min, two tray models" \
  --label "p3_mix_v6=P3 final + tray track kept 8 s after its last observation (a 620 mm tray places its last pocket 6.5 s after leaving the view); cartons + bars 30/min" \
  --label "p5_dwell=P5: release dwell 0.25 s (cup tracks the pocket through the unseal), cartons 60/min" \
  --label "p5_dwell_lag=P5: release dwell 0.25 s + track lag 0.04 s, cartons 60/min" \
  "$@"
