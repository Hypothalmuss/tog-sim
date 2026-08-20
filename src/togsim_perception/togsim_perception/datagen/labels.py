"""Fortress panoptic label maps -> YOLO-seg polygon labels.

labels_map (rgb8): R = semantic label (Gazebo `Label` plugin value), G/B = instance id (hi/lo byte).
tog-sim labels: 1 = product_bar, 2 = product_carton, 3 = tray  ->  YOLO class ids 0, 1, 2.
"""

import cv2
import numpy as np

CLASS_NAMES = ["bar", "carton", "tray"]
LABEL_TO_CLASS = {1: 0, 2: 1, 3: 2}


def instances_from_labels_map(labels: np.ndarray, min_area: int = 60):
    """Yield (class_id, instance_mask[bool]) for every labelled instance in a (H, W, 3) uint8 label map.

    Instance ids are not guaranteed unique across spawned models, so every (class, id) key is additionally split
    into connected components; each component becomes its own instance.
    """
    sem = labels[:, :, 0].astype(np.int32)
    inst = labels[:, :, 1].astype(np.int32) * 256 + labels[:, :, 2].astype(np.int32)
    key = sem * 65536 + inst
    for k in np.unique(key):
        label = int(k // 65536)
        if label not in LABEL_TO_CLASS:
            continue
        m = (key == k).astype(np.uint8)
        n, comp = cv2.connectedComponents(m, connectivity=8)
        for c in range(1, n):
            cm = comp == c
            if int(cm.sum()) >= min_area:
                yield LABEL_TO_CLASS[label], cm


def mask_to_polygons(mask: np.ndarray, min_area: int = 120, eps_px: float = 1.5):
    """Largest external contour of a boolean mask as a list of (x, y) pixel points (or None)."""
    m = mask.astype(np.uint8)
    if int(m.sum()) < min_area:
        return None
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    c = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2)
    if len(c) < 3:
        return None
    return c


def yolo_seg_lines(labels: np.ndarray, min_area: int = 120):
    """YOLO-seg label file content for one label map."""
    h, w = labels.shape[:2]
    lines = []
    for cls, mask in instances_from_labels_map(labels):
        poly = mask_to_polygons(mask, min_area)
        if poly is None:
            continue
        coords = " ".join(f"{x / w:.5f} {y / h:.5f}" for x, y in poly)
        lines.append(f"{cls} {coords}")
    return lines


def overlay(rgb: np.ndarray, labels: np.ndarray, alpha: float = 0.45):
    """Debug overlay: instance masks tinted per class with polygon outlines."""
    out = rgb.copy()
    colors = {0: (255, 200, 0), 1: (255, 60, 60), 2: (60, 200, 255)}
    for cls, mask in instances_from_labels_map(labels):
        poly = mask_to_polygons(mask)
        if poly is None:
            continue
        col = np.array(colors[cls], dtype=np.float32)
        out[mask] = (out[mask] * (1 - alpha) + col * alpha).astype(np.uint8)
        cv2.polylines(out, [poly.reshape(-1, 1, 2)], True, colors[cls], 1)
    return out
