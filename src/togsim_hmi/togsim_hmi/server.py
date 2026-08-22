#!/usr/bin/env python3
"""Operator HMI: a FastAPI app serving a vanilla-JS panel, backed by a small rclpy bridge node.

GET  /api/state          snapshot: cycle status (from /togsim/hmi/status), belts, trays (occupancy), products, vacuum
POST /api/belts          {"infeed": m/s, "outfeed": m/s}
POST /api/run            {"cycles": N, "perception": "vision"|"gt", "profile": ..., "belt": m/s} -> starts run_cycle
POST /api/stop           stops the running cycle (SIGINT)
Run: ros2 run togsim_hmi hmi_server --ros-args -p port:=8080 -p use_sim_time:=true
"""

import json
import os
import signal
import subprocess
import threading
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from togsim_msgs.msg import PickCandidateArray, TrayState, VacuumState


class HmiBridge(Node):
    """Collects everything the panel shows and publishes what it commands."""

    def __init__(self):
        super().__init__("hmi_bridge")
        self.declare_parameter("port", 8080)
        self.lock = threading.Lock()
        self.status = {"state": "idle", "cycles": 0, "attempts": 0, "cpm": 0.0}
        self.belts = {"infeed": 0.0, "outfeed": 0.0}
        self.trays = {}  # id -> dict
        self.products = {"n": 0, "pickable": 0, "t": 0.0}
        self.vacuum = {"sealed": False, "attached": ""}
        self.joints = {}
        self.events = deque(maxlen=60)
        self.proc = None
        self.proc_lines = deque(maxlen=200)
        self.create_subscription(String, "/togsim/hmi/status", self.on_status, 10)
        for b in ("infeed", "outfeed"):
            self.create_subscription(Float64, f"/togsim/conveyor/{b}/state", lambda m, b=b: self._belt(b, m), 5)
        self.belt_pub = {
            b: self.create_publisher(Float64, f"/togsim/conveyor/{b}/cmd_vel", 10) for b in ("infeed", "outfeed")
        }
        self.create_subscription(TrayState, "/togsim/tracks/trays", self.on_tray, 10)
        self.create_subscription(PickCandidateArray, "/togsim/tracks/products", self.on_products, 5)
        self.create_subscription(VacuumState, "/togsim/vacuum/state_msg", self.on_vacuum, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def _belt(self, name, msg):
        with self.lock:
            self.belts[name] = round(float(msg.data), 3)

    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.status.update(st)
            last = st.get("last", "")
            if last and (not self.events or self.events[-1][1] != last):
                self.events.append((time.strftime("%H:%M:%S"), last))

    def on_tray(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.trays[int(msg.tray_id)] = {
                "id": int(msg.tray_id),
                "x": round(p.x, 3),
                "y": round(p.y, 3),
                "rows": int(msg.rows),
                "cols": int(msg.cols),
                "occupied": [bool(v) for v in msg.occupied],
                "t": time.monotonic(),
            }

    def on_products(self, msg):
        with self.lock:
            self.products = {
                "n": len(msg.candidates),
                "pickable": sum(1 for c in msg.candidates if not c.occluded),
                "t": time.monotonic(),
            }

    def on_vacuum(self, msg):
        with self.lock:
            self.vacuum = {"sealed": bool(msg.sealed), "attached": msg.attached_model}

    def on_joints(self, msg):
        with self.lock:
            self.joints = {n: round(v, 3) for n, v in zip(msg.name, msg.position, strict=False)}

    # ---- commands ----
    def set_belts(self, infeed=None, outfeed=None):
        for name, v in (("infeed", infeed), ("outfeed", outfeed)):
            if v is not None:
                self.belt_pub[name].publish(Float64(data=float(v)))
                with self.lock:
                    self.events.append((time.strftime("%H:%M:%S"), f"{name} belt -> {float(v):.2f} m/s"))

    def start_run(self, cycles=20, perception="vision", belt=0.10):
        if self.proc is not None and self.proc.poll() is None:
            return False, "already running"
        cmd = [
            "ros2",
            "run",
            "togsim_task",
            "run_cycle",
            "--ros-args",
            "-p",
            f"perception:={perception}",
            "-p",
            "continuous:=true",
            "-p",
            f"cycles:={int(cycles)}",
            "-p",
            f"belt_speed:={float(belt)}",
            "-p",
            "use_sim_time:=true",
        ]
        self.proc_lines.clear()
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True
        )
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        with self.lock:
            self.status = {"state": "starting", "cycles": 0, "attempts": 0, "cpm": 0.0, "failures": {}}
            self.events.append(
                (time.strftime("%H:%M:%S"), f"run started: {cycles} cycles, {perception}, belt {belt} m/s")
            )
        return True, "started"

    def _pump(self, proc):
        for line in proc.stdout:
            self.proc_lines.append(line.rstrip()[-160:])
        with self.lock:
            if self.status.get("state") not in ("finished",):
                self.status["state"] = "stopped"
            self.events.append((time.strftime("%H:%M:%S"), f"run_cycle exited ({proc.returncode})"))

    def stop_run(self):
        if self.proc is None or self.proc.poll() is not None:
            return False, "not running"
        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        with self.lock:
            self.events.append((time.strftime("%H:%M:%S"), "stop requested"))
        return True, "stopping"

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            trays = [t for t in self.trays.values() if now - t["t"] < 1.5]
            return {
                "status": dict(self.status),
                "running": self.proc is not None and self.proc.poll() is None,
                "belts": dict(self.belts),
                "trays": sorted(({k: v for k, v in t.items() if k != "t"} for t in trays), key=lambda t: -t["x"]),
                "products": {k: v for k, v in self.products.items() if k != "t"}
                if now - self.products["t"] < 1.5
                else {"n": 0, "pickable": 0},
                "vacuum": dict(self.vacuum),
                "joints": dict(self.joints),
                "events": list(self.events)[-20:][::-1],
                "log": list(self.proc_lines)[-12:],
            }


def build_app(bridge):
    from fastapi import FastAPI
    from fastapi.responses import FileResponse, JSONResponse
    from fastapi.staticfiles import StaticFiles

    static = os.path.join(get_package_share_directory("togsim_hmi"), "static")
    app = FastAPI(title="tog-sim HMI")
    app.mount("/static", StaticFiles(directory=static), name="static")

    @app.get("/")
    def index():
        return FileResponse(os.path.join(static, "index.html"))

    @app.get("/api/state")
    def state():
        return JSONResponse(bridge.snapshot())

    @app.post("/api/belts")
    async def belts(body: dict):
        bridge.set_belts(body.get("infeed"), body.get("outfeed"))
        return {"ok": True}

    @app.post("/api/run")
    async def run(body: dict):
        ok, msg = bridge.start_run(
            cycles=int(body.get("cycles", 20)),
            perception=str(body.get("perception", "vision")),
            belt=float(body.get("belt", 0.10)),
        )
        return {"ok": ok, "msg": msg}

    @app.post("/api/stop")
    async def stop():
        ok, msg = bridge.stop_run()
        return {"ok": ok, "msg": msg}

    return app


def main():
    import uvicorn

    rclpy.init()
    bridge = HmiBridge()
    threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()
    port = int(bridge.get_parameter("port").value)
    bridge.get_logger().info(f"HMI at http://0.0.0.0:{port}")
    uvicorn.run(build_app(bridge), host="0.0.0.0", port=port, log_level="warning")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
