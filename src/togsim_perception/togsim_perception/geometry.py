"""Camera geometry helpers shared by the perception nodes (pure numpy, unit-testable)."""

import math

import cv2
import numpy as np


def pixel_to_point(u: float, v: float, depth: float, k: np.ndarray) -> np.ndarray:
    """Back-project a pixel with depth (metres along the optical z axis) using the 3x3 intrinsic matrix."""
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=float)


def point_to_pixel(p: np.ndarray, k: np.ndarray):
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    return fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy


def quat_to_rot(x, y, z, w) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_points(pts: np.ndarray, rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    return pts @ rot.T + trans


def median_depth(depth: np.ndarray, mask: np.ndarray, erode_px: int = 2):
    """Median valid depth under an (eroded) mask; None if too few valid pixels."""
    m = mask.astype(np.uint8)
    if erode_px > 0:
        m = cv2.erode(m, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8))
    vals = depth[m.astype(bool)]
    vals = vals[np.isfinite(vals) & (vals > 0.05)]
    if vals.size < 20:
        return None
    return float(np.median(vals))


def suction_point(mask: np.ndarray):
    """Best cup position: among the pixels with (near-)maximal inscribed-circle radius, the one closest to the
    centroid (elongated products have a whole ridge of maxima). Returns (u, v, radius_px)."""
    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    max_val = float(dist.max())
    ys, xs = np.where(dist >= 0.95 * max_val)
    cy, cx = np.argwhere(mask).mean(axis=0)
    i = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
    return int(xs[i]), int(ys[i]), max_val


def principal_axis(mask: np.ndarray):
    """(centre_u, centre_v, angle_rad, long_side_px, short_side_px) of the mask's minimum-area rectangle.

    The angle is that of the LONG side, in image coordinates (x right, y down), in (-pi/2, pi/2].
    """
    pts = cv2.findNonZero(mask.astype(np.uint8))
    (cu, cv_), (w, h), ang = cv2.minAreaRect(pts)
    ang = math.radians(ang)
    if w < h:
        w, h = h, w
        ang += math.pi / 2
    while ang > math.pi / 2:
        ang -= math.pi
    while ang <= -math.pi / 2:
        ang += math.pi
    return cu, cv_, ang, w, h


def plane_normal(depth: np.ndarray, mask: np.ndarray, k: np.ndarray, step: int = 3):
    """Least-squares plane through the mask's 3D points: (unit normal pointing to the camera, rms residual m)."""
    ys, xs = np.where(mask)
    ys, xs = ys[::step], xs[::step]
    z = depth[ys, xs]
    ok = np.isfinite(z) & (z > 0.05)
    if ok.sum() < 30:
        return None, None
    pts = np.stack([pixel_to_point(u, v, d, k) for u, v, d in zip(xs[ok], ys[ok], z[ok], strict=False)])
    c = pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[2]
    if n[2] > 0:  # optical z points away from the camera; we want the normal facing the camera
        n = -n
    rms = float(s[2] / math.sqrt(len(pts)))
    return n, rms


def polygon_coverage(mask: np.ndarray, polygon_px: np.ndarray) -> float:
    """Fraction of a pixel polygon covered by True pixels of mask."""
    poly = np.zeros(mask.shape, np.uint8)
    cv2.fillPoly(poly, [np.round(polygon_px).astype(np.int32).reshape(-1, 1, 2)], 1)
    area = int(poly.sum())
    if area == 0:
        return 0.0
    return float((poly.astype(bool) & mask).sum()) / area
