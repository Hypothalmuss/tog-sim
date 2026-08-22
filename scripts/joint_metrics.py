#!/usr/bin/env python3
"""Smoothness metrics from /joint_states: per-joint |acceleration| and |jerk| statistics over a sampling window.
The published velocities are resampled onto a uniform 100 Hz grid (message timing is bursty) before differentiating.
Usage: joint_metrics.py [seconds=60] [out.json]"""

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointMetrics(Node):
    def __init__(self):
        super().__init__("joint_metrics")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.samples = []  # (t, {name: velocity})
        self.create_subscription(JointState, "/joint_states", self.on_js, 200)

    def on_js(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((t, dict(zip(msg.name, msg.velocity, strict=False))))


def stats(v):
    if len(v) == 0:
        return {}
    return {
        "p50": round(float(np.percentile(v, 50)), 2),
        "p95": round(float(np.percentile(v, 95)), 2),
        "max": round(float(np.max(v)), 2),
        "n": int(len(v)),
    }


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    out = sys.argv[2] if len(sys.argv) > 2 else ""
    rclpy.init()
    node = JointMetrics()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
    res = {"seconds": secs, "messages": len(node.samples), "acc": {}, "jerk": {}}
    if len(node.samples) > 10:
        ts = np.array([s[0] for s in node.samples])
        order = np.argsort(ts)
        ts = ts[order]
        keep = np.concatenate(([True], np.diff(ts) > 1e-6))  # drop duplicate stamps
        ts = ts[keep]
        grid = np.arange(ts[0], ts[-1], 0.01)
        for name in node.samples[0][1]:
            v = np.array([node.samples[i][1].get(name, 0.0) for i in order])[keep]
            vg = np.interp(grid, ts, v)
            acc = np.gradient(vg, 0.01)
            jerk = np.gradient(acc, 0.01)
            res["acc"][name] = stats(np.abs(acc))
            res["jerk"][name] = stats(np.abs(jerk))
    txt = json.dumps(res, indent=1)
    print(txt)
    if out:
        with open(out, "w") as f:
            f.write(txt)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
