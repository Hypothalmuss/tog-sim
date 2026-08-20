import numpy as np
from togsim_perception.datagen.labels import instances_from_labels_map, yolo_seg_lines


def _labels_map():
    m = np.zeros((60, 80, 3), np.uint8)
    m[10:30, 10:40] = (1, 0, 1)  # bar instance 1
    m[35:55, 50:75] = (2, 0, 7)  # carton instance 7
    m[0:5, 0:5] = (3, 1, 2)  # tiny tray fragment (instance 258) -> filtered by min_area
    return m


def test_instances_and_classes():
    inst = list(instances_from_labels_map(_labels_map()))
    assert sorted(c for c, _ in inst) == [0, 1]  # the 25 px tray fragment is below min_area
    assert sorted(c for c, _ in instances_from_labels_map(_labels_map(), min_area=10)) == [0, 1, 2]


def test_same_instance_id_split_into_components():
    m = np.zeros((60, 80, 3), np.uint8)
    m[5:25, 5:25] = (2, 0, 1)
    m[35:55, 50:75] = (2, 0, 1)  # same (class, id) key, disjoint blob
    inst = list(instances_from_labels_map(m))
    assert len(inst) == 2 and all(c == 1 for c, _ in inst)


def test_yolo_lines_normalised():
    lines = yolo_seg_lines(_labels_map(), min_area=50)
    assert len(lines) == 2
    for ln in lines:
        parts = ln.split()
        assert parts[0] in ("0", "1")
        coords = list(map(float, parts[1:]))
        assert len(coords) >= 6 and all(0.0 <= c <= 1.0 for c in coords)
