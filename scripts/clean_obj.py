#!/usr/bin/env python3
"""Remove empty object/material groups from OBJ files (they crash Gazebo Fortress' Ogre2 mesh import)."""

import sys


def clean(path):
    lines = open(path).read().splitlines()
    out = []
    pending = []  # header lines (o/g/usemtl) waiting for a face
    removed = 0
    for line in lines:
        if line.startswith(("o ", "g ", "usemtl ")):
            # a new group header supersedes pending headers of the same kind that never got faces
            kind = line.split(" ", 1)[0]
            if any(p.startswith(kind + " ") for p in pending):
                removed += 1
                pending = [p for p in pending if not p.startswith(kind + " ")]
            pending.append(line)
        elif line.startswith("f "):
            out.extend(pending)
            pending = []
            out.append(line)
        else:
            out.append(line)
    # drop trailing headers without faces
    removed += sum(1 for p in pending if p.startswith(("o ", "g ")))
    open(path, "w").write("\n".join(out) + "\n")
    return removed


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(f"{p}: removed {clean(p)} empty group header(s)")
