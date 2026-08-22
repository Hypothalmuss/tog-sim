"""Camera geometry helpers shared by the perception nodes (pure numpy, unit-testable)."""

import math

import cv2
import numpy as np


def intrinsics_from_info(width: int, height: int, k_flat, hfov_rad: float) -> np.ndarray:
    """3x3 K from a CameraInfo; falls back to an ideal pinhole from the HFOV when the message is inconsistent
    (Gazebo Fortress' depth camera publishes a 320x240-default K on the shared camera_info topic)."""
    k = np.array(k_flat, dtype=float).reshape(3, 3)
    if abs(k[0, 2] - width / 2.0) > 2.0 or abs(k[1, 2] - height / 2.0) > 2.0 or k[0, 0] < 1.0:
        f = (width / 2.0) / math.tan(hfov_rad / 2.0)
        k = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
    return k


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


def plane_normal(
    depth: np.ndarray, mask: np.ndarray, k: np.ndarray, step: int = 3, erode_px: int = 2, outlier_m: float = 0.005
):
    """Least-squares plane through the mask's 3D points: (unit normal pointing to the camera, rms residual m).

    The mask is eroded first and the fit is repeated without residual outliers, so the belt pixels that a detector
    mask always includes along the edges of a narrow product do not tilt the plane (18 mm of depth step at the
    edges of a 25 mm wide bar used to give ~30 deg of fake tilt)."""
    m = mask.astype(np.uint8)
    if erode_px > 0:
        er = cv2.erode(m, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8))
        if er.sum() >= 30:
            m = er
    ys, xs = np.where(m.astype(bool))
    ys, xs = ys[::step], xs[::step]
    z = depth[ys, xs]
    ok = np.isfinite(z) & (z > 0.05)
    if ok.sum() < 30:
        return None, None
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    pts = np.stack([(xs[ok] - cx) * z[ok] / fx, (ys[ok] - cy) * z[ok] / fy, z[ok]], axis=1)
    n = None
    for _ in range(3):
        c = pts.mean(axis=0)
        _, sv, vt = np.linalg.svd(pts - c, full_matrices=False)
        n = vt[2]
        res = (pts - c) @ n
        keep = np.abs(res) < outlier_m
        if keep.sum() < 30 or keep.all():
            break
        pts = pts[keep]
    if n[2] > 0:  # optical z points away from the camera; we want the normal facing the camera
        n = -n
    c = pts.mean(axis=0)
    rms = float(np.sqrt(np.mean(((pts - c) @ n) ** 2)))
    return n, rms


def axis_tilt(depth: np.ndarray, mask: np.ndarray, k: np.ndarray, axis_rad: float, bins: int = 12, erode_px: int = 1):
    """Tilt of a product about the axis perpendicular to its long axis, from the depth *ridge* profile along that axis.

    Bars are rounded: across their ~25 mm width the depth varies by ~10 mm, which makes a plane fit through the whole
    mask meaningless. Along the long axis the closest-to-camera line (the ridge, 10th depth percentile per bin) is
    straight for a flat product and sloped for a leaning one. Returns (tilt_rad, rms_m), or (0.0, None) when the
    mask holds too few valid pixels."""
    m = mask.astype(np.uint8)
    if erode_px > 0:
        er = cv2.erode(m, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8))
        if er.sum() >= 60:
            m = er
    ys, xs = np.where(m.astype(bool))
    z = depth[ys, xs]
    ok = np.isfinite(z) & (z > 0.05)
    if ok.sum() < 60:
        return 0.0, None
    xs, ys, z = xs[ok], ys[ok], z[ok]
    s = xs * math.cos(axis_rad) + ys * math.sin(axis_rad)  # position along the long axis, px
    edges = np.linspace(s.min(), s.max() + 1e-6, bins + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, bins - 1)
    pos, ridge = [], []
    for b in range(bins):
        sel = idx == b
        if sel.sum() >= 4:
            pos.append(s[sel].mean())
            ridge.append(np.percentile(z[sel], 10))
    if len(pos) < 4:
        return 0.0, None
    pos, ridge = np.array(pos), np.array(ridge)
    gsd = float(np.median(ridge)) / k[0, 0]  # metres per pixel at the product's depth
    a, b = np.polyfit(pos * gsd, ridge, 1)  # depth = a * s + b  ->  slope a = tan(tilt)
    rms = float(np.sqrt(np.mean((a * pos * gsd + b - ridge) ** 2)))
    return float(math.atan(a)), rms


def contact_depth(depth: np.ndarray, mask: np.ndarray, u: int, v: int, half: int = 3):
    """Median depth of the mask pixels in a (2*half+1)^2 window around the suction point: the height the cup meets."""
    y0, y1 = max(v - half, 0), min(v + half + 1, depth.shape[0])
    x0, x1 = max(u - half, 0), min(u + half + 1, depth.shape[1])
    win = depth[y0:y1, x0:x1][mask[y0:y1, x0:x1]]
    win = win[np.isfinite(win) & (win > 0.05)]
    return float(np.median(win)) if win.size else None


def fit_pocket_lattice(points, offsets, init_xy, init_yaw, iters: int = 4, gate_m: float = 0.03):
    """Fit the known pocket grid to observed pocket-floor centres (2-D Procrustes with nearest-neighbour assignment).

    points: (N, 2) observed pocket centres in the world, offsets: (M, 2) pocket centres in the tray frame.
    Returns (x, y, yaw, n_matched, rms_m); with fewer than 3 matches the initial pose is returned with n_matched < 3."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    offs = np.asarray(offsets, dtype=float).reshape(-1, 2)
    x, y, yaw = float(init_xy[0]), float(init_xy[1]), float(init_yaw)
    matched, rms = 0, 0.0
    for _ in range(iters):
        c, s_ = math.cos(yaw), math.sin(yaw)
        pred = np.stack([x + c * offs[:, 0] - s_ * offs[:, 1], y + s_ * offs[:, 0] + c * offs[:, 1]], axis=1)
        pairs = []  # greedy unique assignment, nearest first
        d = np.linalg.norm(pts[:, None, :] - pred[None, :, :], axis=2)
        used_p, used_o = set(), set()
        for k in np.argsort(d, axis=None):
            i, j = divmod(int(k), len(offs))
            if d[i, j] > gate_m:
                break
            if i in used_p or j in used_o:
                continue
            used_p.add(i)
            used_o.add(j)
            pairs.append((i, j))
        matched = len(pairs)
        if matched < 3:
            return x, y, yaw, matched, rms
        P = pts[[i for i, _ in pairs]]
        O = offs[[j for _, j in pairs]]
        pm, om = P.mean(axis=0), O.mean(axis=0)
        H = (O - om).T @ (P - pm)
        u, _, vt = np.linalg.svd(H)
        R = vt.T @ u.T
        if np.linalg.det(R) < 0:  # reflection guard
            vt[1, :] *= -1
            R = vt.T @ u.T
        yaw = math.atan2(R[1, 0], R[0, 0])
        t = pm - R @ om
        x, y = float(t[0]), float(t[1])
        res = P - (t + (R @ O.T).T)
        rms = float(np.sqrt(np.mean(np.sum(res**2, axis=1))))
    # a rectangular grid is symmetric under 180 deg: keep the yaw branch closest to the initial (mask-based) yaw so the
    # pocket indices stay consistent with it
    if abs((yaw - float(init_yaw) + math.pi) % (2 * math.pi) - math.pi) > math.pi / 2:
        yaw = (yaw + math.pi + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw, matched, rms


def polygon_coverage(mask: np.ndarray, polygon_px: np.ndarray) -> float:
    """Fraction of a pixel polygon covered by True pixels of mask."""
    poly = np.zeros(mask.shape, np.uint8)
    cv2.fillPoly(poly, [np.round(polygon_px).astype(np.int32).reshape(-1, 1, 2)], 1)
    area = int(poly.sum())
    if area == 0:
        return 0.0
    return float((poly.astype(bool) & mask).sum()) / area
