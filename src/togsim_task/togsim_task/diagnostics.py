"""Ground-truth evaluation of the cycle: placement metric and the diagnostics that compare vision / motion with
Gazebo's pose/info. Metrics only - nothing here is used for control; `eval:=false` disables the subscription.

The methods were lifted from run_cycle unchanged; attributes they use that belong to the node (lock, logger, clock,
joints, tracked trays, default tray spec, FK) are delegated through __getattr__.
"""

import math
import time

from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from togsim_task.scheduler import PRODUCT_HEIGHT


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tilt_of(q):
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))))


class GroundTruthEval:
    def __init__(self, node: Node, enabled: bool = True):
        self.node = node
        self.enabled = enabled
        self.gt = {}  # name -> ((x, y, z), yaw, tilt, monotonic t)
        self.gt_hist = {}  # tray name -> [(sim t, x, y)]
        self.gt_tool = {}  # tool-like links, if Gazebo reports them
        self.placements = []  # (offset_m, yaw_err_deg) per placed product
        if enabled:
            node.create_subscription(TFMessage, "/world/cell/pose/info", self.on_gt, 10, callback_group=node.cb)

    def __getattr__(self, name):  # node attributes used by the lifted methods
        return getattr(self.node, name)

    def on_gt(self, msg):
        now = time.monotonic()
        if not getattr(self, "_gt_names_logged", False) and len(msg.transforms) > 20:
            self._gt_names_logged = True
            names = sorted(
                {t.child_frame_id for t in msg.transforms if not t.child_frame_id.startswith(("product_", "tray_"))}
            )
            self.get_logger().info(f"GT frames ({len(msg.transforms)}): {names[:60]}")
        with self.lock:
            for t in msg.transforms:
                n = t.child_frame_id
                if "tcp" in n or "vgc10_suction" in n:  # the tool itself, if Gazebo reports links
                    tr = t.transform.translation
                    self.gt_tool[n] = (tr.x, tr.y, tr.z, now)
                if n.startswith(("product_", "tray_")):
                    tr, q = t.transform.translation, t.transform.rotation
                    prev = self.gt.get(n)
                    if n.startswith("tray_") and prev is not None and abs(tr.y - prev[0][1]) > 0.01:
                        self.get_logger().warn(
                            f"{n} shoved: y {prev[0][1]:.3f} -> {tr.y:.3f} at x {tr.x:.3f}, sim {self.sim_now():.2f},"
                            f" during '{self.cur_goal}'"
                        )
                    self.gt[n] = ((tr.x, tr.y, tr.z), yaw_of(q), tilt_of(q), now)
                    if n.startswith("tray_"):
                        h = self.gt_hist.setdefault(n, [])
                        h.append((self.sim_now(), tr.x, tr.y))
                        del h[:-60]
            for n in [k for k, v in self.gt.items() if now - v[3] > 2.0]:
                self.gt.pop(n, None)

    def in_tray_frame(self, pp, gt=None):
        """(tray name, x, y) of world point pp in the frame of the nearest GT tray, or None."""
        if gt is None:
            with self.lock:
                gt = dict(self.gt)
        best = None
        for tn, (tp, tyaw, _, _) in gt.items():
            if not tn.startswith("tray_"):
                continue
            dx, dy = pp[0] - tp[0], pp[1] - tp[1]
            d = math.hypot(dx, dy)
            if best is None or d < best[0]:
                c, s_ = math.cos(-tyaw), math.sin(-tyaw)
                best = (d, tn, c * dx - s_ * dy, s_ * dx + c * dy)
        return None if best is None else best[1:]

    def frame_err_now(self, tid):
        """Diagnostic: the tracked tray frame (latest message, predicted to now) vs the GT tray, as text."""
        with self.lock:
            ent = self.vis_trays.get(tid)
        if ent is None:
            return ""
        m = ent[0]
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        dt = min(1.0, max(0.0, self.sim_now() - st))
        loc = self.in_tray_frame((m.pose.pose.position.x + m.velocity.x * dt, m.pose.pose.position.y))
        return "" if loc is None else f"; frame err at release ({1e3 * loc[1]:+.0f},{1e3 * loc[2]:+.0f}) mm vs {loc[0]}"

    def fk_vs_gt(self, gt_xy):
        """Diagnostic: planar FK of the measured joints (model l1=0.40, l2=0.25, base at the world origin) vs a GT
        point that should coincide with the cup (the product centre while it is held), with the elbow sign."""
        with self.lock:
            js = dict(self.joints)
        j1 = next((v for n, v in js.items() if n.endswith("j1_joint")), None)
        j2 = next((v for n, v in js.items() if n.endswith("j2_joint")), None)
        if j1 is None or j2 is None:
            return ""
        x = 0.40 * math.cos(j1) + 0.25 * math.cos(j1 + j2)
        y = 0.40 * math.sin(j1) + 0.25 * math.sin(j1 + j2)
        with self.lock:
            tools = dict(self.gt_tool)
        tool_txt = ""
        for n, (tx, ty, _tz, _t) in tools.items():
            tool_txt += f", FK-GT[{n.split('/')[-1].split('::')[-1]}] ({1e3 * (x - tx):+.0f},{1e3 * (y - ty):+.0f}) mm"
        return (
            f"; FK-GT ({1e3 * (x - gt_xy[0]):+.0f},{1e3 * (y - gt_xy[1]):+.0f}) mm, elbow {'+' if j2 >= 0 else '-'}"
            f" j1 {math.degrees(j1):.0f} j2 {math.degrees(j2):.0f} deg{tool_txt}"
        )

    def measure_placement(self, key, tray, pocket, held=""):
        """Offset of the product the gripper just released (`held`, GT name) from the nearest pocket centre of the tray
        it sits on, 0.4 s after release. Index-free: the vision tray yaw may be 180 deg from the GT convention."""
        # pose at release (before the sleep) vs 0.4 s later: separates a bad placement from release dynamics
        with self.lock:
            g0 = self.gt.get(held)
        time.sleep(0.4)
        with self.lock:
            gt = dict(self.gt)
            g1 = gt.get(held)
        if g0 is not None and g1 is not None:
            dmove = math.hypot(g1[0][0] - g0[0][0], g1[0][1] - g0[0][1])
            dyaw = math.degrees((g1[1] - g0[1] + math.pi) % (2 * math.pi) - math.pi)
            self.get_logger().info(
                f"release dynamics {held}: moved {1e3 * dmove:.0f} mm, turned {dyaw:+.0f} deg in 0.4 s"
            )
        trays = {n: v for n, v in gt.items() if n.startswith("tray_")}
        best = None
        for tn, (tp, tyaw, _, _) in trays.items():
            spec = (tray.spec if tray is not None else None) or self.tray
            centres = [spec.pocket_world(tp, tyaw, k) for k in range(spec.n_pockets)]
            for pn, (pp, pyaw, _, _) in gt.items():
                if not pn.startswith("product_") or abs(pp[2] - tp[2]) > 0.06 or (held and pn != held):
                    continue
                if math.hypot(pp[0] - tp[0], pp[1] - tp[1]) > 0.25:
                    continue  # not on this tray
                for k, (px, py, _) in enumerate(centres):
                    d = math.hypot(pp[0] - px, pp[1] - py)
                    if best is None or d < best[0]:
                        best = (d, pn, math.degrees((pyaw - tyaw + math.pi / 2) % math.pi - math.pi / 2), tn, k)
        if best is None:
            return
        if best[0] >= 0.05:
            loc = self.in_tray_frame(gt[best[1]][0], gt)
            where = "" if loc is None else f"; product at ({1e3 * loc[1]:+.0f},{1e3 * loc[2]:+.0f}) mm in GT {loc[0]}"
            self.get_logger().warn(
                f"placed {key} -> {tray.key}[{pocket}]: nearest product {1e3 * best[0]:.0f} mm from any pocket centre"
                f" (not counted){where}"
            )
            return
        self.placements.append((best[0], abs(best[2])))
        self.get_logger().info(
            f"placed {key} -> {tray.key}[{pocket}] ({best[1]} in {best[3]}[{best[4]}]): "
            f"{1e3 * best[0]:.1f} mm off centre, yaw {best[2]:+.1f} deg"
        )

    def report_placement(self):
        if not self.placements:
            return
        d = sorted(x[0] for x in self.placements)
        y = sorted(x[1] for x in self.placements)
        self.get_logger().info(
            f"placement accuracy over {len(d)}: offset mean {1e3 * sum(d) / len(d):.1f} mm, "
            f"p95 {1e3 * d[int(0.95 * (len(d) - 1))]:.1f} mm | yaw mean {sum(y) / len(y):.1f} deg, "
            f"p95 {y[int(0.95 * (len(y) - 1))]:.1f} deg"
        )

    # ---------------- per-phase diagnostics (text for the log; "" / None without ground truth) ----------------
    def occupancy_gt(self, t):
        """Pocket occupancy of the GT tray nearest to tracked tray `t`, as 'x..x (tray_N)', or '?'."""
        with self.lock:
            gt = dict(self.gt)
        loc = self.in_tray_frame((t.x, t.y), gt)
        if loc is None:
            return "?"
        tp, tyaw = gt[loc[0]][0], gt[loc[0]][1]
        spec = t.spec or self.tray
        occ = [False] * spec.n_pockets
        for pn, (pp, _, _, _) in gt.items():
            if pn.startswith("product_") and abs(pp[2] - tp[2]) < 0.06:
                k = spec.pocket_of_point(tp, tyaw, pp[0], pp[1])
                if k is not None:
                    occ[k] = True
        return "".join("x" if v else "." for v in occ) + f" ({loc[0]})"

    def nearest_product(self, x, y):
        """(distance, name, dx, dy, entry) of the GT product nearest to the estimate (x, y), or None."""
        with self.lock:
            gt = dict(self.gt)
        return min(
            (
                (math.hypot(g[0][0] - x, g[0][1] - y), n, g[0][0] - x, g[0][1] - y, g)
                for n, g in gt.items()
                if n.startswith("product_")
            ),
            default=None,
        )

    def no_seal_detail(self, c, contact_gt, xe):
        """Why a pick did not seal: was the estimate wrong (top height, position) or the product unpickable?
        `contact_gt` is nearest_product() at contact, `xe` the estimate's x at the failure."""
        why = ""
        if contact_gt is not None:
            _d, nm, dx, dy, _g = contact_gt
            why = f"; at contact GT {nm} was ({1e3 * dx:+.0f},{1e3 * dy:+.0f}) mm from the estimate"
        near = self.nearest_product(xe, c.y)
        if near is not None:
            d, n, _dx, _dy, g = near
            hgt = PRODUCT_HEIGHT.get("product_" + n.split("_")[1], 0.0)
            why += (
                f"; at failure nearest GT {n} {1e3 * d:.0f} mm from the estimate, top est {1e3 * c.top:.1f} vs GT"
                f" {1e3 * (g[0][2] + hgt):.1f} mm, tilt {math.degrees(g[2]):.0f} deg"
            )
        return why

    def log_grasp_offset(self, c, held, tp, psi):
        """Where the cup really sealed relative to the product centre (GT) vs the offset the place compensates for
        (`c.off`, measured from the track `tp` and FK), the pick yaw error and the cup command error."""
        with self.lock:
            g = self.gt.get(held)
        if g is None or tp is None:
            return
        ax, ay = g[0][0] - tp.x, g[0][1] - tp.y
        dyaw = math.degrees((c.yaw - g[1] + math.pi / 2) % math.pi - math.pi / 2)  # mod 180: axis ambiguity is fine
        cupd = (
            ""
            if psi is None
            else f", cup-cmd {math.degrees((psi - c.yaw + math.pi) % (2 * math.pi) - math.pi):+.0f} deg"
        )
        self.get_logger().info(
            f"grasp offset {held}: actual ({1e3 * ax:+.0f},{1e3 * ay:+.0f}) mm, estimated"
            f" ({1e3 * c.off[0]:+.0f},{1e3 * c.off[1]:+.0f}) mm, pick yaw error {dyaw:+.0f} deg{cupd}"
        )

    def tray_frame_detail(self, tray):
        """The tracked tray frame vs the real tray now, and the same message against GT at its own stamp (separates
        a stale delivery from a lagging track)."""
        loc = self.in_tray_frame((tray.x, tray.y))
        if loc is None:
            return ""
        ferr = f", frame err ({1e3 * loc[1]:+.0f},{1e3 * loc[2]:+.0f}) mm vs {loc[0]}"
        st = getattr(tray, "stamp", None)
        if st is not None:
            with self.lock:
                hist = list(self.gt_hist.get(loc[0], []))
            if hist:
                tg, xg, _ = min(hist, key=lambda e: abs(e[0] - st))
                ferr += (
                    f"; stamp age {self.sim_now() - st:.2f} s, raw x - GT x at stamp {1e3 * (tray.x_raw - xg):+.0f} mm"
                    f" (GT dt {tg - st:+.2f}), vx {tray.vx:.3f}, predicted +{1e3 * (tray.x - tray.x_raw):.0f} mm,"
                    f" rx age {getattr(tray, 'rx_age', -1.0):.2f} s"
                )
        return ferr

    def log_release_yaw(self, held, tray, pocket, target, psi_seal):
        """Who is rotated at release, the cup or the product? Cup vs tray, product vs tray and product vs cup (GT),
        where the product is in the GT tray, and how far J4 turned since the seal."""
        psi = self.cup_yaw()
        with self.lock:
            g = self.gt.get(held)
        if psi is None or g is None:
            return
        wrap = lambda a: math.degrees((a + math.pi) % (2 * math.pi) - math.pi)  # noqa: E731
        wrap180 = lambda a: math.degrees((a + math.pi / 2) % math.pi - math.pi / 2)  # noqa: E731
        loc = self.in_tray_frame(g[0])
        where = "" if loc is None else f", product at ({1e3 * loc[1]:+.0f},{1e3 * loc[2]:+.0f}) mm in GT {loc[0]}"
        self.get_logger().info(
            f"release yaw {held} -> {tray.key}[{pocket}]: cup-tray {wrap(psi - tray.yaw):+.0f} deg,"
            f" product-tray {wrap180(g[1] - tray.yaw):+.0f} deg, product-cup {wrap180(g[1] - psi):+.0f} deg"
            f"{where}; target pocket ({1e3 * target[0]:+.0f},{1e3 * target[1]:+.0f}) mm in {tray.key}"
            + (
                ""
                if psi_seal is None
                else f"; J4 turned {wrap(psi - psi_seal):+.0f} deg since seal"
                + self.frame_err_now(tray.tid)
                + self.fk_vs_gt(g[0])
            )
        )
