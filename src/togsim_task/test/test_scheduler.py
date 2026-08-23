import math

from togsim_task.scheduler import (
    CandidateView,
    TrayView,
    candidate_reason,
    free_pocket,
    nearest_branch,
    pocket_order,
    reachable,
)
from togsim_task.trays import TraySpec

SPEC = TraySpec("tray_2x4", 2, 4, 0.09, 0.07, 0.08, 0.06, 0.025, 0.010)


def test_reachable_annulus():
    assert reachable(0.4, 0.3)
    assert not reachable(0.1, 0.1)  # folded against the base
    assert not reachable(0.6, 0.3)  # stretched


def test_nearest_branch_minimises_turn():
    assert abs(nearest_branch(0.0, math.radians(170)) - math.pi) < 1e-9
    assert abs(nearest_branch(0.0, math.radians(80))) < 1e-9
    assert abs(nearest_branch(math.radians(10), math.radians(-100)) - math.radians(-170)) < 1e-9


def test_pocket_order_leading_first():
    order = pocket_order(SPEC)
    xs = [SPEC.pocket_offset(k)[0] for k in order]
    assert xs == sorted(xs, reverse=True)
    assert order[0] in (3, 7) and order[-1] in (0, 4)


def test_free_pocket_prefers_leading_reachable_and_skips_occupied():
    tray = TrayView("t1", 0.25, 0.35, 0.52, 0.0, 0.09, [False] * 8)
    k = free_pocket(SPEC, tray, lead=1.2, margin=0.3)
    assert k is not None
    # at x 0.25 the leading column (+0.135) is still in reach at arrival (0.49 m, r 0.58): it goes first
    assert SPEC.pocket_offset(k)[0] == SPEC.pocket_offset(3)[0]
    occ = [False] * 8
    occ[k] = True
    tray2 = TrayView("t1", 0.25, 0.35, 0.52, 0.0, 0.09, occ)
    k2 = free_pocket(SPEC, tray2, lead=1.2, margin=0.3)
    assert k2 is not None and k2 != k


def test_free_pocket_none_when_tray_has_left_reach():
    tray = TrayView("t1", 0.70, 0.35, 0.52, 0.0, 0.09, [False] * 8)
    assert free_pocket(SPEC, tray, lead=1.2, margin=0.3) is None


def test_free_pocket_respects_latch():
    tray = TrayView("t1", 0.25, 0.35, 0.52, 0.0, 0.09, [False] * 8)
    k = free_pocket(SPEC, tray, lead=1.2, margin=0.3)
    k2 = free_pocket(SPEC, tray, lead=1.2, margin=0.3, latched={("t1", k): 10.0}, now=5.0)
    assert k2 is not None and k2 != k


def test_candidate_reasons():
    base = dict(cls="product_carton", x=0.2, y=-0.3, top=0.545, vx=0.1, occluded=False, tilt=0.0)
    ok = CandidateView("v1", **base)
    assert candidate_reason(ok, 0.9, (0.10, 0.50), ["product_carton"], 0.52) is None
    assert (
        candidate_reason(CandidateView("v2", **{**base, "occluded": True}), 0.9, (0.1, 0.5), ["product_carton"], 0.52)
        == "occluded"
    )
    assert (
        candidate_reason(
            CandidateView("v3", **{**base, "cls": "product_bar"}), 0.9, (0.1, 0.5), ["product_carton"], 0.52
        )
        == "no tray for class"
    )
    assert (
        candidate_reason(CandidateView("v4", **{**base, "x": 0.6}), 0.9, (0.1, 0.5), ["product_carton"], 0.52)
        == "window"
    )
    assert (
        candidate_reason(CandidateView("v5", **{**base, "top": 0.7}), 0.9, (0.1, 0.5), ["product_carton"], 0.52)
        == "height"
    )
    assert (
        candidate_reason(ok, 0.9, (0.1, 0.5), ["product_carton"], 0.52, blacklist={"v1": 9.0}, now=5.0) == "blacklist"
    )
