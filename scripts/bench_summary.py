#!/usr/bin/env python3
"""Summarise a run_cycle log (+ sim log) into JSON: cycles, attempts, cpm, motion, placement, timeline, failures."""

import json
import re
import sys


def main():
    log = open(sys.argv[1]).read() if len(sys.argv) > 1 else ""
    sim = open(sys.argv[2]).read() if len(sys.argv) > 2 else ""
    d = {
        "cycles": 0,
        "attempts": 0,
        "cpm": 0.0,
        "motion_mean_s": 0.0,
        "placement_mm_mean": None,
        "placement_mm_p95": None,
        "yaw_deg_mean": None,
        "timeline": {},
        "failures": {},
        "sim_failed_calls": len(re.findall(r"failed \(attempt", sim)),
    }
    m = re.search(r"finished: (\d+) cycles, (\d+) attempts", log)
    if m:
        d["cycles"], d["attempts"] = int(m.group(1)), int(m.group(2))
    last = re.findall(r"cycle \d+: [0-9.]+ s \| ([0-9.]+) cpm \| mean motion ([0-9.]+) s", log)
    if last:
        d["cpm"], d["motion_mean_s"] = float(last[-1][0]), float(last[-1][1])
    m = re.search(
        r"placement accuracy over \d+: offset mean ([0-9.]+) mm, p95 ([0-9.]+) mm \| yaw mean ([0-9.]+) deg", log
    )
    if m:
        d["placement_mm_mean"], d["placement_mm_p95"], d["yaw_deg_mean"] = (
            float(m.group(1)),
            float(m.group(2)),
            float(m.group(3)),
        )
    m = re.search(r"timeline means (.*)", log)
    if m:
        parts = m.group(1).split()
        d["timeline"] = {parts[i]: float(parts[i + 1]) for i in range(0, len(parts) - 1, 2)}
    for w in re.findall(r"\[WARN\] \[[0-9.]+\] \[run_cycle\]: ([a-z ]+?)(?: v\d+| on v\d+|:)", log):
        d["failures"][w.strip()] = d["failures"].get(w.strip(), 0) + 1
    print(json.dumps(d, indent=2))


if __name__ == "__main__":
    main()
