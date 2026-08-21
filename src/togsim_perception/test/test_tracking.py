import math

from togsim_perception.tracking import BeltTracker, Observation


def _frame(t, objs, vx=0.1):
    return [Observation(x + vx * t, y, 0.5386, yaw, cls, t, vx=vx) for (x, y, yaw, cls) in objs]


def test_ids_are_stable_on_a_moving_belt():
    tr = BeltTracker()
    objs = [(0.0, -0.35, 0.3, "product_bar"), (0.15, -0.30, -1.0, "product_carton")]
    ids = None
    for k in range(20):
        t = k * 0.1
        touched = tr.update(_frame(t, objs))
        got = sorted((round(x.x - 0.1 * t, 3), x.id) for x in touched)
        if ids is None:
            ids = [i for _, i in got]
        assert [i for _, i in got] == ids, "ids must not change while the objects keep moving"
    assert len(tr.tracks) == 2


def test_prediction_extrapolates_with_belt_velocity():
    tr = BeltTracker()
    for k in range(5):
        tr.update(_frame(k * 0.1, [(0.0, -0.35, 0.0, "product_bar")]))
    (p,) = tr.predict(0.4 + 0.5)
    assert abs(p.x - (0.0 + 0.1 * 0.9)) < 0.003
    assert abs(p.y + 0.35) < 1e-6


def test_velocity_is_refined_from_observations_when_encoder_is_off():
    tr = BeltTracker(vel_alpha=0.5)
    # encoder says 0.10 but the object really moves at 0.12 m/s
    for k in range(12):
        t = k * 0.1
        tr.update([Observation(0.12 * t, -0.35, 0.54, 0.0, "product_bar", t, vx=0.10)])
    (p,) = tr.predict(1.1)
    assert abs(p.vx - 0.12) < 0.01
    assert abs(p.x - 0.12 * 1.1) < 0.005


def test_missed_frames_and_loss():
    tr = BeltTracker(lost_s=0.5)
    tr.update(_frame(0.0, [(0.0, -0.35, 0.0, "product_bar")]))
    tr.update(_frame(0.3, [(0.0, -0.35, 0.0, "product_bar")]))  # gap of 0.3 s: still the same track
    assert len(tr.tracks) == 1
    tr.update([], t_now=1.0)  # nothing seen for 0.7 s: lost
    assert len(tr.tracks) == 0


def test_keyed_observations_override_geometry():
    tr = BeltTracker(gate_m=0.02)
    a = Observation(0.0, -0.35, 0.54, 0.0, "product_bar", 0.0, 0.1, key="product_bar_1")
    b = Observation(0.0, -0.35, 0.54, 0.0, "product_bar", 0.1, 0.1, key="product_bar_1")  # jumped 1 cm more than gate
    b.x = 0.05
    ta = tr.update([a])[0]
    tb = tr.update([b])[0]
    assert ta.id == tb.id


def test_yaw_fused_modulo_symmetry():
    tr = BeltTracker()
    tr.update([Observation(0.0, -0.35, 0.54, math.radians(85), "product_bar", 0.0)])
    (t,) = tr.update([Observation(0.0, -0.35, 0.54, math.radians(-88), "product_bar", 0.1)])  # same axis, flipped sign
    assert abs(abs(_deg(t.yaw)) - 88.5) < 2.0 or abs(abs(_deg(t.yaw)) - 91.5) < 2.0


def _deg(a):
    return math.degrees(a)


def test_predict_drops_tracks_when_the_source_stops():
    tr = BeltTracker(lost_s=1.0)
    tr.update(_frame(0.0, [(0.0, -0.35, 0.0, "product_bar")]))
    assert len(tr.predict(0.5)) == 1
    assert tr.predict(5.0) == []  # no update() calls any more: predict must not extrapolate a dead track for ever
