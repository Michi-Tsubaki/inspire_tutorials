from pathlib import Path
from xml.etree import ElementTree

from inspire_tutorials.hand_mapping import LEFT_JOINTS, RIGHT_JOINTS
from inspire_tutorials.urdf_builder import build_inspire_hands_robot_description


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_DIR = ROOT / "description" / "inspire_hand"


def _robot(hand):
    return ElementTree.fromstring(
        build_inspire_hands_robot_description(DESCRIPTION_DIR, hand)
    )


def _names(root, tag):
    return [element.attrib["name"] for element in root.findall(tag)]


def test_right_sim_urdf_contains_only_right_actuated_joints():
    root = _robot("right")
    joint_names = set(_names(root, "joint"))
    for joint_name in RIGHT_JOINTS:
        assert joint_name in joint_names
    for joint_name in LEFT_JOINTS:
        assert joint_name not in joint_names
    assert root.find("./ros2_control/joint[@name='R_index_proximal_joint']") is not None


def test_both_sim_urdf_prefixes_links_joints_mimics_and_meshes():
    root = _robot("both")
    link_names = _names(root, "link")
    joint_names = _names(root, "joint")

    assert len(link_names) == len(set(link_names))
    assert len(joint_names) == len(set(joint_names))
    assert "R_hand_base_link" in link_names
    assert "L_hand_base_link" in link_names
    assert "R_thumb_proximal_pitch_joint" in joint_names
    assert "L_thumb_proximal_pitch_joint" in joint_names

    mimic_joints = [mimic.attrib["joint"] for mimic in root.findall(".//mimic")]
    assert "R_thumb_proximal_pitch_joint" in mimic_joints
    assert "L_thumb_proximal_pitch_joint" in mimic_joints

    mesh_filenames = [
        mesh.attrib["filename"]
        for mesh in root.findall(".//mesh")
        if "filename" in mesh.attrib
    ]
    assert mesh_filenames
    assert all(
        filename.startswith("package://inspire_tutorials/description/inspire_hand/")
        for filename in mesh_filenames
    )


def test_ros2_control_uses_same_action_controller_joint_names_as_real_g1():
    root = _robot("both")
    control_joints = {
        joint.attrib["name"] for joint in root.findall("./ros2_control/joint")
    }
    assert set(RIGHT_JOINTS) <= control_joints
    assert set(LEFT_JOINTS) <= control_joints
