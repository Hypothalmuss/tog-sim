"""Pure scheduling rules of the continuous pick-and-place cycle (no ROS): reach, pocket choice, candidate gating.

Everything here is a function of plain numbers so it can be unit-tested and tuned without a simulator. `run_cycle`
calls these with its tracked products / trays; the numbers are the cell's (robot at the world origin, belts along x).
"""

import math
from dataclasses import dataclass

REACH_MIN, REACH_MAX = 0.22, 0.62  # m, tilt pivot radius the GX8 reaches comfortably
PLACE_X_WINDOW = (
    -0.20,
    0.52,
)  # m, pocket x at arrival: the reach checks bound the far side, this keeps the cup off the base
MIN_RADIUS_NOW = 0.30  # m, not folded against the base, now and at arrival

# the cell's products and trays (heights: the cup contact height above the belt is product top - contact offset)
PRODUCT_HEIGHT = {"product_bar": 0.0186, "product_carton": 0.025}
PRODUCT_LENGTH = {"product_bar": 0.168, "product_carton": 0.07}  # m, long side
PRODUCT_FITS = {"tray_2x4": ["product_carton"], "tray_bar_2x3": ["product_bar"]}
BELT_Z = 0.52  # m, belt surface in the world


@dataclass
class CycleConfig:
    """The constants of one continuous run, read once from the parameters at start."""

    belt_speed: float
    hover: float  # m above the product top where the tracked descent starts
    x_window: tuple  # m, pick window along the infeed (x at arrival)
    lead: float  # s, pick look-ahead: the fly-through point is predicted this far ahead
    place_lead: float  # s, place look-ahead
    seal_budget: float  # s, dwell at contact waiting for the seal (the place goal preempts it)
    contact: float  # m, contact offset below the product top (press)
    seal_timeout: float  # s, give the product up when the cup does not seal
    fits: list  # every product class a known tray takes
    park: list  # m, x y z of the park pose
    track_lag: float = 0.04  # s, the measured arm trails a tracked pocket by belt speed x this (place lead)
    release_dwell: float = 0.25  # s, tracked dwell at the pocket after the release command (covers the unseal delay)


def reachable(x, y, rmin=REACH_MIN, rmax=REACH_MAX):
    """Inside the annulus the arm reaches without stretching or folding."""
    return rmin < math.hypot(x, y) < rmax


def nearest_branch(yaw, ref, period=math.pi):
    """Representative of `yaw` modulo `period` closest to `ref`: products and pockets are symmetric under 180 deg, so
    the cup only ever needs to turn by at most 90 deg."""
    return yaw + round((ref - yaw) / period) * period


def pocket_order(spec):
    """Leading pockets first (largest x in the tray frame, then the row nearest the centre line): they leave the reach
    first, and a tray that passes with empty trailing pockets is cheaper than one with empty leading ones."""
    return sorted(range(spec.n_pockets), key=lambda k: (-spec.pocket_offset(k)[0], abs(spec.pocket_offset(k)[1])))


@dataclass
class TrayView:
    """What the scheduler needs to know about a tracked tray."""

    key: str
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    occupied: list


def free_pocket(spec, tray, lead, margin=0.5, latched=None, now=0.0, window=PLACE_X_WINDOW):
    """First free, unlatched pocket that is reachable when the cup arrives (`lead` s from now) and still `margin` s
    later (settling the heading can make the place outlast the lead); None if there is none."""
    latched = latched or {}
    for i in pocket_order(spec):
        if tray.occupied[i] or latched.get((tray.key, i), 0.0) > now:
            continue
        px, py, _ = spec.pocket_world((tray.x + tray.vx * lead, tray.y, tray.z), tray.yaw, i)
        px0, py0, _ = spec.pocket_world((tray.x, tray.y, tray.z), tray.yaw, i)
        px1, py1, _ = spec.pocket_world((tray.x + tray.vx * (lead + margin), tray.y, tray.z), tray.yaw, i)
        if (
            reachable(px, py)
            and reachable(px1, py1)
            and window[0] < px < window[1]
            and math.hypot(px, py) > MIN_RADIUS_NOW
            and math.hypot(px0, py0) > MIN_RADIUS_NOW
        ):
            return i
    return None


@dataclass
class CandidateView:
    """What the scheduler needs to know about a tracked product."""

    key: str
    cls: str
    x: float
    y: float
    top: float
    vx: float
    occluded: bool
    tilt: float


def candidate_reason(c, lead, x_window, fits, belt_z, blacklist=None, now=0.0, y_window=(-0.6, -0.1), settle_s=1.3):
    """Why a product is not pickable right now, or None if it is. `lead` = flight time to it; the product must still be
    in reach `settle_s` later (cup settled and sealed)."""
    blacklist = blacklist or {}
    x_arrive = c.x + c.vx * lead
    x_done = x_arrive + c.vx * settle_s
    if c.occluded:
        return "occluded"
    if c.cls not in fits:
        return "no tray for class"
    if not (x_window[0] < x_arrive < x_window[1]):
        return "window"
    if not (y_window[0] < c.y < y_window[1]):
        return "y"
    if not (reachable(x_done, c.y) and math.hypot(x_arrive, c.y) > 0.28 and math.hypot(c.x, c.y) > MIN_RADIUS_NOW):
        return "reach"
    if not (belt_z + 0.008 < c.top < belt_z + 0.08):
        return "height"
    if c.tilt >= math.radians(4):
        return "tilt"
    if c.key in blacklist and blacklist[c.key] >= now:
        return "blacklist"
    return None
