"""Client for the persistent Ignition service bridge (`ign_service_bridge`, service /togsim/ign_service).

Replaces per-call `ign service` subprocesses: one discovery, reliable replies, verified creates/removes.
"""

import math
import re

from togsim_msgs.srv import IgnService


def q_from_rpy(r, p, y):
    cy, sy, cp, sp, cr, sr = (
        math.cos(y / 2),
        math.sin(y / 2),
        math.cos(p / 2),
        math.sin(p / 2),
        math.cos(r / 2),
        math.sin(r / 2),
    )
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class IgnClient:
    def __init__(self, node, world="cell", timeout_s=5.0, callback_group=None):
        self.node = node
        self.world = world
        self.timeout_s = float(timeout_s)
        self.cli = node.create_client(IgnService, "/togsim/ign_service", callback_group=callback_group)

    def wait(self, timeout_s=30.0):
        return self.cli.wait_for_service(timeout_sec=timeout_s)

    def call(self, service, reqtype, reptype, request, timeout_s=None):
        """Blocking call (spin-safe: uses the future, the node must be spun by another thread/executor)."""
        req = IgnService.Request()
        req.service, req.request_type, req.response_type, req.request = service, reqtype, reptype, request
        req.timeout_s = float(timeout_s or self.timeout_s)
        fut = self.cli.call_async(req)
        deadline = req.timeout_s + 2.0
        import time

        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > deadline:
                return False, "bridge timeout"
            time.sleep(0.002)
        res = fut.result()
        return bool(res.success), res.response

    # ---------------- convenience ----------------
    def create(self, sdf_filename, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        qx, qy, qz, qw = q_from_rpy(roll, pitch, yaw)
        req = (
            f'sdf_filename: "{sdf_filename}", name: "{name}", allow_renaming: false, '
            f"pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
            f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}"
        )
        ok, _ = self.call(f"/world/{self.world}/create", "ignition.msgs.EntityFactory", "ignition.msgs.Boolean", req)
        return ok

    def remove(self, name):
        ok, _ = self.call(
            f"/world/{self.world}/remove",
            "ignition.msgs.Entity",
            "ignition.msgs.Boolean",
            f'name: "{name}", type: MODEL',
        )
        return ok

    def models(self, prefix=""):
        """Names of the models in the world (from the scene info service); None if the query failed."""
        ok, text = self.call(
            f"/world/{self.world}/scene/info", "ignition.msgs.Empty", "ignition.msgs.Scene", "", timeout_s=10.0
        )
        if not ok:
            return None
        names = []
        for block in re.split(r"\nmodel \{", text):
            m = re.search(r'^\s*name: "([^"]+)"', block, re.M)
            if m and m.group(1).startswith(prefix):
                names.append(m.group(1))
        return names

    def set_light(self, text):
        ok, _ = self.call(f"/world/{self.world}/light_config", "ignition.msgs.Light", "ignition.msgs.Boolean", text)
        return ok
