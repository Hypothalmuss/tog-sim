"""Tray pocket geometry shared by the task layer and (later) the perception layer."""

import math
from dataclasses import dataclass


@dataclass
class TraySpec:
    name: str
    rows: int
    cols: int
    pitch_x: float
    pitch_y: float
    pocket_x: float
    pocket_y: float
    pocket_depth: float
    base_thickness: float = 0.004

    @classmethod
    def from_yaml(cls, cfg: dict, name: str) -> "TraySpec":
        t = cfg["trays"][name]
        return cls(
            name,
            t["rows"],
            t["cols"],
            t["pitch_x"],
            t["pitch_y"],
            t["pocket_x"],
            t["pocket_y"],
            t["pocket_depth"],
            t.get("base_thickness", 0.004),
        )

    @property
    def n_pockets(self) -> int:
        return self.rows * self.cols

    def pocket_offset(self, index: int):
        """Pocket centre (x, y) in the tray frame (origin = tray bottom centre). Row-major, cols along +x."""
        r, c = divmod(index, self.cols)
        x = -self.cols * self.pitch_x / 2 + (c + 0.5) * self.pitch_x
        y = -self.rows * self.pitch_y / 2 + (r + 0.5) * self.pitch_y
        return x, y

    def pocket_world(self, tray_xyz, tray_yaw, index: int):
        """Pocket centre in the world frame (x, y, z_floor)."""
        ox, oy = self.pocket_offset(index)
        c, s = math.cos(tray_yaw), math.sin(tray_yaw)
        return (tray_xyz[0] + c * ox - s * oy, tray_xyz[1] + s * ox + c * oy, tray_xyz[2] + self.base_thickness)

    def pocket_of_point(self, tray_xyz, tray_yaw, px, py):
        """Index of the pocket containing world point (px, py), or None."""
        c, s = math.cos(tray_yaw), math.sin(tray_yaw)
        dx, dy = px - tray_xyz[0], py - tray_xyz[1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        col = int(math.floor((lx + self.cols * self.pitch_x / 2) / self.pitch_x))
        row = int(math.floor((ly + self.rows * self.pitch_y / 2) / self.pitch_y))
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return row * self.cols + col
        return None
