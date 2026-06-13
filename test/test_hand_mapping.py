import json
from pathlib import Path

import pytest

from inspire_tutorials.hand_mapping import (
    CLOSED_POSITIONS,
    FINGER_LANDMARKS,
    LEFT_JOINTS,
    OPEN_POSITIONS,
    RIGHT_JOINTS,
    XYZ,
    joint_names_for_side,
    landmarks_to_closure_fractions,
    landmarks_to_positions,
    normalize_hand_selection,
    normalize_handedness,
    percentages_to_positions,
    preset_percentages,
)


ROOT = Path(__file__).resolve().parents[1]


def _fixture(name):
    data = json.loads((ROOT / "test_data" / name).read_text())
    return [XYZ(*row) for row in data]


def _with_pip_bent(finger):
    points = list(_fixture("open_palm_landmarks.json"))
    mcp, pip, dip, tip = FINGER_LANDMARKS[finger]
    base = points[mcp]
    points[pip] = XYZ(base.x, base.y - 0.12, base.z)
    points[dip] = XYZ(base.x + 0.10, base.y - 0.12, base.z)
    points[tip] = XYZ(base.x + 0.20, base.y - 0.12, base.z)
    return points


def test_open_palm_and_closed_fist_are_not_inverted():
    open_positions = landmarks_to_positions(_fixture("open_palm_landmarks.json"))
    closed_positions = landmarks_to_positions(_fixture("closed_fist_landmarks.json"))

    for open_value, closed_value in zip(open_positions, closed_positions):
        assert closed_value > open_value

    assert max(open_positions) < 0.2
    assert closed_positions[0] > 0.18
    assert closed_positions[1] > 0.35
    assert min(closed_positions[2:]) > 1.2


def test_closure_fractions_are_bounded_for_dummy_open_and_fist_data():
    open_fractions = landmarks_to_closure_fractions(_fixture("open_palm_landmarks.json"))
    closed_fractions = landmarks_to_closure_fractions(
        _fixture("closed_fist_landmarks.json")
    )

    assert all(0.0 <= value <= 1.0 for value in open_fractions)
    assert all(0.0 <= value <= 1.0 for value in closed_fractions)
    assert max(open_fractions) < 0.15
    assert closed_fractions[1] > 0.95
    assert min(closed_fractions[2:]) > 0.95


def test_video_fist_closes_both_thumb_joints():
    fractions = landmarks_to_closure_fractions(_fixture("video_fist_landmarks.json"))

    assert fractions[0] > 0.85
    assert fractions[1] > 0.85
    assert min(fractions[2:]) > 0.95


def test_non_thumb_flexion_uses_pip_not_only_dip_angle():
    closure_index_by_finger = {
        "index": 2,
        "middle": 3,
        "ring": 4,
        "pinky": 5,
    }
    for finger, closure_index in closure_index_by_finger.items():
        fractions = landmarks_to_closure_fractions(_with_pip_bent(finger))
        assert fractions[closure_index] > 0.75
        other_fingers = [
            value
            for index, value in enumerate(fractions[2:], start=2)
            if index != closure_index
        ]
        assert max(other_fingers) < 0.15


def test_percentage_commands_map_zero_to_open_and_hundred_to_closed():
    assert percentages_to_positions([0]) == pytest.approx(OPEN_POSITIONS)
    assert percentages_to_positions([100]) == pytest.approx(CLOSED_POSITIONS)

    mixed = percentages_to_positions([50, 0, 100, 25, 75, 10])
    assert mixed[0] == pytest.approx(CLOSED_POSITIONS[0] * 0.5)
    assert mixed[1] == pytest.approx(0.0)
    assert mixed[2] == pytest.approx(CLOSED_POSITIONS[2])

    ordered = percentages_to_positions([10, 20, 30, 40, 50, 60])
    assert ordered == pytest.approx(
        [
            CLOSED_POSITIONS[0] * 0.1,
            CLOSED_POSITIONS[1] * 0.2,
            CLOSED_POSITIONS[2] * 0.3,
            CLOSED_POSITIONS[3] * 0.4,
            CLOSED_POSITIONS[4] * 0.5,
            CLOSED_POSITIONS[5] * 0.6,
        ]
    )


def test_named_presets_match_g1_interface_examples():
    assert preset_percentages("start_grasp") == (40.0, 20.0, 80.0, 80.0, 80.0, 80.0)
    assert preset_percentages("default-grasp") == (
        10.0,
        30.0,
        100.0,
        100.0,
        100.0,
        100.0,
    )


def test_side_and_handedness_contracts():
    assert normalize_hand_selection("right") == ("right",)
    assert normalize_hand_selection("left") == ("left",)
    assert normalize_hand_selection("both") == ("right", "left")
    assert joint_names_for_side("right") == RIGHT_JOINTS
    assert joint_names_for_side("left") == LEFT_JOINTS
    assert normalize_handedness("Right") == "right"
    assert normalize_handedness("Right", mirror_handedness=True) == "left"

    with pytest.raises(ValueError):
        normalize_hand_selection("arm")
