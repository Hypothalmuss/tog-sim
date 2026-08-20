#!/usr/bin/env python3
"""Regenerate YOLO-seg label files of a dataset from its saved raw label maps (after a converter change).

Usage: relabel_dataset.py ~/togsim_data/seg_v1
"""

import glob
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "togsim_perception"))
from togsim_perception.datagen.labels import yolo_seg_lines  # noqa: E402


def main():
    root = os.path.expanduser(sys.argv[1])
    n = 0
    for raw in sorted(glob.glob(os.path.join(root, "labels_raw", "*.png"))):
        labels = cv2.cvtColor(cv2.imread(raw), cv2.COLOR_BGR2RGB)
        stem = os.path.splitext(os.path.basename(raw))[0]
        lines = yolo_seg_lines(labels)
        with open(os.path.join(root, "labels", stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        n += 1
    print(f"relabelled {n} files in {root}")


if __name__ == "__main__":
    main()
