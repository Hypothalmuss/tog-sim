"""The xacro must expand to a valid URDF with the expected kinematic chain."""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

EXPECTED_JOINTS = ["j1_joint", "j2_joint", "j3_joint", "j4_joint", "tilt_joint"]


def _expand(sim: str) -> ET.Element:
    share = get_package_share_directory("togsim_description")
    xacro_file = os.path.join(share, "urdf", "togsim_robot.urdf.xacro")
    out = subprocess.check_output(["xacro", xacro_file, f"sim:={sim}"], text=True)
    return ET.fromstring(out)


def test_chain_and_limits():
    root = _expand("false")
    joints = {j.get("name"): j for j in root.findall("joint")}
    for name in EXPECTED_JOINTS:
        assert name in joints, f"missing joint {name}"
    assert joints["j3_joint"].get("type") == "prismatic"
    assert joints["tilt_joint"].find("axis").get("xyz") == "1 0 0"
    # Epson datasheet velocity for J1 survives into the URDF
    assert abs(float(joints["j1_joint"].find("limit").get("velocity")) - 10.053) < 1e-2
    # every non-fixed link that carries geometry has an inertial block (Gazebo requirement)
    for link in root.findall("link"):
        if link.find("visual") is not None:
            assert link.find("inertial") is not None, f"{link.get('name')} has no inertial"
    assert any(link.get("name") == "tcp" for link in root.findall("link"))


def test_check_urdf_passes():
    root = _expand("true")
    tmp = "/tmp/togsim_robot_test.urdf"
    ET.ElementTree(root).write(tmp)
    subprocess.check_call(["check_urdf", tmp], stdout=subprocess.DEVNULL)
