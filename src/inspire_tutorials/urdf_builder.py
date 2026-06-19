"""Build a standalone Inspire hand simulation URDF from vendored dex-urdf."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence
from xml.etree import ElementTree

from inspire_tutorials.hand_mapping import (
    CLOSED_POSITIONS,
    OPEN_POSITIONS,
    joint_names_for_side,
    normalize_hand_selection,
)


ACTUATED_BASENAMES = (
    "thumb_proximal_yaw_joint",
    "thumb_proximal_pitch_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)


def _prefix_name(prefix: str, value: str) -> str:
    if value.startswith(prefix):
        return value
    return f"{prefix}{value}"


def _rewrite_mesh_filename(filename: str) -> str:
    if "://" in filename:
        return filename
    return f"package://inspire_tutorials/description/inspire_hand/{filename}"


def _prefixed_hand_elements(
    source_urdf: Path,
    prefix: str,
) -> Sequence[ElementTree.Element]:
    source_root = ElementTree.parse(source_urdf).getroot()
    elements = []
    for child in list(source_root):
        copied = ElementTree.fromstring(ElementTree.tostring(child, encoding="unicode"))
        if copied.tag == "link" and "name" in copied.attrib:
            copied.set("name", _prefix_name(prefix, copied.attrib["name"]))
        if copied.tag == "joint":
            copied.set("name", _prefix_name(prefix, copied.attrib["name"]))
            parent = copied.find("parent")
            child_link = copied.find("child")
            mimic = copied.find("mimic")
            if parent is not None and "link" in parent.attrib:
                parent.set("link", _prefix_name(prefix, parent.attrib["link"]))
            if child_link is not None and "link" in child_link.attrib:
                child_link.set("link", _prefix_name(prefix, child_link.attrib["link"]))
            if mimic is not None and "joint" in mimic.attrib:
                mimic.set("joint", _prefix_name(prefix, mimic.attrib["joint"]))

        for mesh in copied.findall(".//mesh"):
            filename = mesh.attrib.get("filename")
            if filename:
                mesh.set("filename", _rewrite_mesh_filename(filename))
        elements.append(copied)
    return elements


def _add_fixed_mount(
    robot: ElementTree.Element,
    side: str,
    prefix: str,
) -> None:
    y = "-0.14" if side == "right" else "0.14"
    joint = ElementTree.SubElement(
        robot,
        "joint",
        {"name": f"world_to_{prefix.rstrip('_')}_base", "type": "fixed"},
    )
    ElementTree.SubElement(joint, "parent", {"link": "world"})
    ElementTree.SubElement(joint, "child", {"link": f"{prefix}base"})
    ElementTree.SubElement(joint, "origin", {"xyz": f"0 {y} 0", "rpy": "0 0 0"})


def _add_ros2_control(robot: ElementTree.Element, sides: Iterable[str]) -> None:
    control = ElementTree.SubElement(
        robot,
        "ros2_control",
        {"name": "InspireHandsMockSystem", "type": "system"},
    )
    hardware = ElementTree.SubElement(control, "hardware")
    ElementTree.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"

    for side in sides:
        for joint_name, open_pos, _closed_pos in zip(
            joint_names_for_side(side), OPEN_POSITIONS, CLOSED_POSITIONS
        ):
            joint = ElementTree.SubElement(control, "joint", {"name": joint_name})
            command = ElementTree.SubElement(joint, "command_interface", {"name": "position"})
            ElementTree.SubElement(command, "param", {"name": "min"}).text = "0.0"
            ElementTree.SubElement(command, "param", {"name": "max"}).text = "1.7"
            state = ElementTree.SubElement(joint, "state_interface", {"name": "position"})
            ElementTree.SubElement(state, "param", {"name": "initial_value"}).text = str(open_pos)
            ElementTree.SubElement(joint, "state_interface", {"name": "velocity"})


def build_inspire_hands_robot_description(description_dir: str | Path, hand: str) -> str:
    """Return robot_description XML for selected left/right Inspire hands."""

    base_dir = Path(description_dir)
    sides = normalize_hand_selection(hand)
    robot = ElementTree.Element("robot", {"name": f"inspire_hand_{hand}_sim"})
    ElementTree.SubElement(robot, "link", {"name": "world"})

    for side in sides:
        prefix = "R_" if side == "right" else "L_"
        source = base_dir / f"inspire_hand_{side}.urdf"
        _add_fixed_mount(robot, side, prefix)
        for element in _prefixed_hand_elements(source, prefix):
            robot.append(element)

    _add_ros2_control(robot, sides)
    return '<?xml version="1.0"?>\n' + ElementTree.tostring(
        robot,
        encoding="unicode",
    )
