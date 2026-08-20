import math

from togsim_task.trays import TraySpec


def spec():
    return TraySpec("t", rows=2, cols=4, pitch_x=0.09, pitch_y=0.07, pocket_x=0.08, pocket_y=0.06, pocket_depth=0.025)


def test_pocket_offsets_are_centred():
    t = spec()
    xs = [t.pocket_offset(i)[0] for i in range(t.n_pockets)]
    ys = [t.pocket_offset(i)[1] for i in range(t.n_pockets)]
    assert abs(sum(xs)) < 1e-9 and abs(sum(ys)) < 1e-9
    assert abs(t.pocket_offset(0)[0] - (-0.135)) < 1e-9 and abs(t.pocket_offset(0)[1] - (-0.035)) < 1e-9


def test_pocket_roundtrip_with_yaw():
    t = spec()
    tray = (0.4, 0.35, 0.52)
    for yaw in (0.0, 0.3, -1.2, math.pi / 2):
        for i in range(t.n_pockets):
            x, y, z = t.pocket_world(tray, yaw, i)
            assert t.pocket_of_point(tray, yaw, x, y) == i
            assert abs(z - 0.524) < 1e-9
    assert t.pocket_of_point(tray, 0.0, 10.0, 0.0) is None
