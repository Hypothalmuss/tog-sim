"""Constant-velocity multi-object tracking on a conveyor (pure Python, unit-testable).

Observations come in bursts (one camera frame) with an observation time; tracks are predicted to any later time with
the belt velocity. Association is greedy nearest-neighbour within a gate, per class; an observation may carry a stable
`key` (ground-truth model name) which then overrides geometric association.
"""

import math
from dataclasses import dataclass, field


@dataclass
class Observation:
    x: float
    y: float
    z: float
    yaw: float
    cls: str
    t: float  # observation time, seconds
    vx: float = 0.0  # belt velocity at observation (world frame)
    vy: float = 0.0
    key: str | None = None  # stable identity if known (GT mode)
    payload: object = None  # anything the consumer wants back (e.g. the original message)


@dataclass
class Track:
    id: int
    cls: str
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    t: float  # time the state refers to
    n_obs: int = 1
    key: str | None = None
    payload: object = None
    yaw_period: float = math.pi
    history: list = field(default_factory=list)  # (t, x, y) of the last observations, for velocity estimation

    def predicted(self, t: float):
        """(x, y) extrapolated to time t."""
        dt = t - self.t
        return self.x + self.vx * dt, self.y + self.vy * dt

    def age(self, t: float) -> float:
        return t - self.t


def _wrap(a: float, period: float) -> float:
    return (a + period / 2) % period - period / 2


class BeltTracker:
    def __init__(
        self,
        gate_m: float = 0.06,
        lost_s: float = 1.0,
        pos_alpha: float = 0.6,
        vel_alpha: float = 0.3,
        yaw_period: float = math.pi,
        estimate_velocity: bool = True,
        min_obs_velocity: int = 3,
    ):
        self.gate = gate_m
        self.lost_s = lost_s
        self.pos_alpha = pos_alpha  # weight of the new observation in the position update
        self.vel_alpha = vel_alpha  # weight of the measured velocity in the velocity update
        self.yaw_period = yaw_period
        self.estimate_velocity = estimate_velocity
        self.min_obs_velocity = min_obs_velocity
        self.tracks: dict[int, Track] = {}
        self._next_id = 1

    # ---------------- update ----------------
    def update(self, obs: list[Observation], t_now: float | None = None) -> list[Track]:
        """Fuse one frame of observations (all at about the same time). Returns the tracks touched by this frame."""
        if not obs:
            if t_now is not None:
                self._prune(t_now)
            return []
        t = obs[0].t
        touched = []
        unmatched_tracks = set(self.tracks)
        # 1) keyed observations (ground truth): exact identity
        remaining = []
        for o in obs:
            if o.key is None:
                remaining.append(o)
                continue
            tr = next((tr for tr in self.tracks.values() if tr.key == o.key), None)
            if tr is None:
                tr = self._create(o)
            else:
                self._fuse(tr, o)
                unmatched_tracks.discard(tr.id)
            touched.append(tr)
        # 2) geometric association for the rest: nearest first within the gate, same class
        pairs = []
        for i, o in enumerate(remaining):
            for tid in unmatched_tracks:
                tr = self.tracks[tid]
                if tr.cls != o.cls or tr.key is not None:
                    continue
                px, py = tr.predicted(o.t)
                d = math.hypot(o.x - px, o.y - py)
                if d <= self.gate:
                    pairs.append((d, i, tid))
        pairs.sort()
        used_obs = set()
        for _d, i, tid in pairs:
            if i in used_obs or tid not in unmatched_tracks:
                continue
            used_obs.add(i)
            unmatched_tracks.discard(tid)
            self._fuse(self.tracks[tid], remaining[i])
            touched.append(self.tracks[tid])
        for i, o in enumerate(remaining):
            if i not in used_obs:
                touched.append(self._create(o))
        self._prune(t if t_now is None else max(t, t_now))
        return touched

    def predict(self, t: float) -> list[Track]:
        """Snapshot of all live tracks extrapolated to time t (new Track objects, originals untouched).
        Tracks that have not been observed for `lost_s` are dropped here too (the source may have died)."""
        self._prune(t)
        out = []
        for tr in self.tracks.values():
            x, y = tr.predicted(t)
            out.append(
                Track(tr.id, tr.cls, x, y, tr.z, tr.yaw, tr.vx, tr.vy, t, tr.n_obs, tr.key, tr.payload, tr.yaw_period)
            )
        return out

    # ---------------- internals ----------------
    def _create(self, o: Observation) -> Track:
        tr = Track(self._next_id, o.cls, o.x, o.y, o.z, o.yaw, o.vx, o.vy, o.t, 1, o.key, o.payload, self.yaw_period)
        tr.history.append((o.t, o.x, o.y))
        self.tracks[tr.id] = tr
        self._next_id += 1
        return tr

    def _fuse(self, tr: Track, o: Observation):
        dt = o.t - tr.t
        if dt < 0:  # out-of-order frame: ignore the stale one
            return
        px, py = tr.predicted(o.t)
        a = self.pos_alpha
        tr.x = (1 - a) * px + a * o.x
        tr.y = (1 - a) * py + a * o.y
        tr.z = (1 - a) * tr.z + a * o.z
        tr.yaw = tr.yaw + a * _wrap(o.yaw - tr.yaw, tr.yaw_period)
        tr.t = o.t
        tr.n_obs += 1
        tr.payload = o.payload
        tr.history.append((o.t, o.x, o.y))
        if len(tr.history) > 10:
            tr.history.pop(0)
        # velocity: the belt encoder value is the prior; refined from the observed displacement once there is history
        vx, vy = o.vx, o.vy
        if self.estimate_velocity and len(tr.history) >= self.min_obs_velocity:
            (t0, x0, y0), (t1, x1, y1) = tr.history[0], tr.history[-1]
            span = t1 - t0
            if span > 0.15:
                mvx, mvy = (x1 - x0) / span, (y1 - y0) / span
                if math.hypot(mvx - o.vx, mvy - o.vy) < 0.05:  # sane: within 5 cm/s of the encoder
                    vx, vy = mvx, mvy
        b = self.vel_alpha
        tr.vx = (1 - b) * tr.vx + b * vx
        tr.vy = (1 - b) * tr.vy + b * vy

    def _prune(self, t: float):
        for tid in [tid for tid, tr in self.tracks.items() if tr.age(t) > self.lost_s]:
            del self.tracks[tid]
