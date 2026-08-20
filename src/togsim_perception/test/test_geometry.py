import math

import numpy as np
from togsim_perception.geometry import pixel_to_point, point_to_pixel, polygon_coverage, principal_axis, suction_point

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
