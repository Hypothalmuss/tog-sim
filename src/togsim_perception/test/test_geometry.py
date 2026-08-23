import math

import numpy as np

from togsim_perception.geometry import (
    axis_tilt,
    contact_depth,
    fit_pocket_lattice,
    pixel_to_point,
    plane_normal,
    point_to_pixel,
    polygon_coverage,
    principal_axis,
    suction_point,
)

K = np.array([[600.0, 0, 424.0], [0, 600.0, 240.0], [0, 0, 1.0]])


def test_pixel_point_roundtrip():
    p = pixel_to_point(500, 300, 1.1, K)
    u, v = point_to_pixel(p, K)
    assert abs(u - 500) < 1e-9 and abs(v - 300) < 1e-9 and abs(p[2] - 1.1) < 1e-12


def test_suction_point_is_centre_of_rectangle():
    m = np.zeros((100, 200), bool)
    m[20:60, 50:150] = True
    u, v, r = suction_point(m)
    assert 95 <= u <= 105 and 35 <= v <= 45 and 18 <= r <= 21


def test_principal_axis_angle():
    m = np.zeros((200, 200), bool)
    m[90:110, 40:160] = True  # long along x -> angle ~ 0
    cu, cv, ang, long_px, short_px = principal_axis(m)
    assert abs(ang) < math.radians(3) and long_px > short_px
    m2 = m.T.copy()  # long along y -> +-90 deg
    _, _, ang2, _, _ = principal_axis(m2)
    assert abs(abs(ang2) - math.pi / 2) < math.radians(3)


def test_polygon_coverage():
    m = np.zeros((50, 50), bool)
    m[10:30, 10:30] = True
    poly = np.array([[10, 10], [30, 10], [30, 30], [10, 30]], float)
    assert 0.9 < polygon_coverage(m, poly) <= 1.0
    assert polygon_coverage(m, poly + 30) == 0.0


def test_intrinsics_fallback():
    from togsim_perception.geometry import intrinsics_from_info

    bad = [277.0, 0, 160.0, 0, 277.0, 120.0, 0, 0, 1]
    k = intrinsics_from_info(848, 480, bad, 1.2043)
    assert abs(k[0, 2] - 424) < 1e-9 and abs(k[1, 2] - 240) < 1e-9 and 600 < k[0, 0] < 630
    good = [615.0, 0, 424.0, 0, 615.0, 240.0, 0, 0, 1]
    assert abs(intrinsics_from_info(848, 480, good, 1.2043)[0, 0] - 615.0) < 1e-9


def test_plane_normal_ignores_belt_pixels_at_mask_edges():
    """A flat 25 mm bar seen from 1 m: the detector mask is 2 px too wide on each side and therefore contains belt
    pixels that are 18.6 mm farther away; the fitted normal must still be (anti)parallel to the optical axis."""
    depth = np.full((480, 848), 1.0, np.float32)  # belt plane at 1 m
    bar = np.zeros((480, 848), bool)
    bar[230:250, 300:420] = True  # ~20 px wide (25 mm at 600 px focal length), 120 px long
    depth[bar] = 1.0 - 0.0186
    mask = bar.copy()
    mask[228:252, 298:422] = True  # mask bleeds 2 px onto the belt all around
    n, rms = plane_normal(depth, mask, K)
    tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
    assert tilt < 1.0, tilt
    assert rms < 0.002
    # a genuinely tilted surface is still measured
    u = np.arange(848, dtype=np.float32)[None, :]
    depth2 = 1.0 - 0.0186 - (u - 300) * (1.0 / 600.0) * math.tan(math.radians(20.0))  # 20 deg about the image y axis
    depth2 = np.broadcast_to(depth2, (480, 848)).copy()
    n2, _ = plane_normal(depth2, bar, K, erode_px=0)
    tilt2 = math.degrees(math.acos(min(1.0, abs(n2[2]))))
    assert abs(tilt2 - 20.0) < 2.0, tilt2


def _rounded_bar(depth_top=1.0814, length_px=120, width_px=20, slope=0.0):
    """Depth image of a rounded bar on a belt at 1.1 m; `slope` = metres of depth change per metre along the bar."""
    depth = np.full((480, 848), 1.1, np.float32)
    mask = np.zeros((480, 848), bool)
    u = np.arange(300, 300 + length_px)
    for dv in range(-width_px // 2, width_px // 2):
        v = 240 + dv
        sag = 0.010 * (abs(dv) / (width_px / 2)) ** 2  # rounded cross-section: 10 mm lower at the edges
        depth[v, u] = depth_top + sag + slope * (u - 300) * (depth_top / 600.0)
        mask[v, u] = True
    return depth, mask


def test_axis_tilt_flat_rounded_bar_is_zero():
    depth, mask = _rounded_bar()
    tilt, rms = axis_tilt(depth, mask, K, 0.0)
    assert abs(math.degrees(tilt)) < 1.0
    assert rms < 0.001
    # a plane fit across the rounded width is NOT reliable - that is why axis_tilt exists
    # (documented behaviour, not asserted)


def test_axis_tilt_measures_leaning_bar():
    depth, mask = _rounded_bar(slope=math.tan(math.radians(25.0)))
    tilt, _ = axis_tilt(depth, mask, K, 0.0)
    assert abs(math.degrees(tilt) - 25.0) < 2.0, math.degrees(tilt)


def test_contact_depth_is_ridge_not_mask_median():
    depth, mask = _rounded_bar()
    su, sv, _ = suction_point(mask)
    assert abs(contact_depth(depth, mask, su, sv) - 1.0814) < 0.002
    assert float(np.median(depth[mask])) > 1.0814 + 0.002  # the mask median sits lower on the rounded flanks


def test_fit_pocket_lattice_recovers_pose_from_partial_noisy_pockets():
    rows, cols, px, py = 2, 4, 0.09, 0.07
    offs = [
        (-cols * px / 2 + (c + 0.5) * px, -rows * py / 2 + (r + 0.5) * py) for r in range(rows) for c in range(cols)
    ]
    tx, ty, tyaw = 0.31, 0.352, math.radians(6.0)
    cy, sy = math.cos(tyaw), math.sin(tyaw)
    rng = np.random.default_rng(1)
    pts = [
        (tx + cy * ox - sy * oy + rng.normal(0, 0.002), ty + sy * ox + cy * oy + rng.normal(0, 0.002))
        for ox, oy in offs
    ]
    pts = [p for i, p in enumerate(pts) if i not in (2, 5)]  # two pockets occupied / not visible
    pts.append((tx + 0.2, ty + 0.2))  # an outlier blob
    x, y, yaw, n, rms = fit_pocket_lattice(pts, offs, (tx + 0.015, ty - 0.012), tyaw + math.radians(4))
    assert n == 6
    assert abs(x - tx) < 0.003 and abs(y - ty) < 0.003
    assert abs(math.degrees(yaw - tyaw)) < 1.0
    assert rms < 0.004


def test_fit_pocket_lattice_keeps_the_initial_yaw_branch():
    rows, cols, px, py = 2, 4, 0.09, 0.07
    offs = [
        (-cols * px / 2 + (c + 0.5) * px, -rows * py / 2 + (r + 0.5) * py) for r in range(rows) for c in range(cols)
    ]
    tx, ty, tyaw = 0.2, 0.35, math.radians(3.0)
    cy, sy = math.cos(tyaw), math.sin(tyaw)
    pts = [(tx + cy * ox - sy * oy, ty + sy * ox + cy * oy) for ox, oy in offs]
    # start from the 180-degree-rotated guess: the symmetric grid fits either way, the result must follow the guess
    x, y, yaw, n, _ = fit_pocket_lattice(pts, offs, (tx, ty), tyaw + math.pi)
    assert n == 8 and abs(x - tx) < 1e-6 and abs(y - ty) < 1e-6
    assert abs(((yaw - (tyaw + math.pi) + math.pi) % (2 * math.pi)) - math.pi) < math.radians(1)


def test_fit_pocket_lattice_per_axis_gate_takes_a_large_error_along_a_long_pitch():
    # a 2x3 bar tray (200 x 65 mm pitch): the mask centroid of a long tray can be 60 mm off along it; the isotropic
    # 30 mm gate finds no pocket, the per-axis gate (0.45 pitch) recovers the pose and still refuses a lateral error
    rows, cols, px, py = 2, 3, 0.200, 0.065
    offs = [
        (-cols * px / 2 + (c + 0.5) * px, -rows * py / 2 + (r + 0.5) * py) for r in range(rows) for c in range(cols)
    ]
    tx, ty, tyaw = 0.26, -0.45, math.radians(2.0)
    cy, sy = math.cos(tyaw), math.sin(tyaw)
    pts = [(tx + cy * ox - sy * oy, ty + sy * ox + cy * oy) for ox, oy in offs]
    gate = (0.45 * px, 0.45 * py)
    x, y, yaw, n, rms = fit_pocket_lattice(pts, offs, (tx + 0.060, ty + 0.004), tyaw, gate_xy=gate)
    assert n == 6 and abs(x - tx) < 0.002 and abs(y - ty) < 0.002 and rms < 0.002
    _x, _y, _yaw, n_iso, _rms = fit_pocket_lattice(pts, offs, (tx + 0.060, ty + 0.004), tyaw)
    assert n_iso < 3
    # across the tray the gate is below half the short pitch: an error inside it snaps to the right row (beyond it
    # any lattice fit is ambiguous, which is why the mask centroid anchors the fit and the node gates the correction)
    _x, y_lat, _yaw, n_lat, _rms = fit_pocket_lattice(pts, offs, (tx, ty + 0.025), tyaw, gate_xy=gate)
    assert n_lat == 6 and abs(y_lat - ty) < 0.002


def test_suction_point_prefers_the_given_centre_on_a_ridge():
    # a bar mask truncated at one end: its centroid is off the true centre; with the detector's box centre preferred,
    # the suction point sits at the true centre instead of following the centroid
    mask = np.zeros((100, 200), bool)
    mask[45:56, 40:121] = True  # visible part of a bar whose true extent is u=40..160 (true centre u=100)
    u0, _v0, _r = suction_point(mask)
    u1, v1, _r = suction_point(mask, prefer=(100.0, 50.0))
    assert abs(u0 - 80) <= 2 and abs(u1 - 100) <= 2 and v1 == 50
