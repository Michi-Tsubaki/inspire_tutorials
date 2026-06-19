"""MediaPipe hand landmark to Inspire RH56DFX joint mapping."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Mapping, Sequence, Tuple


SIDES = ("right", "left")

RIGHT_JOINTS = (
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
)

LEFT_JOINTS = (
    "L_thumb_proximal_yaw_joint",
    "L_thumb_proximal_pitch_joint",
    "L_index_proximal_joint",
    "L_middle_proximal_joint",
    "L_ring_proximal_joint",
    "L_pinky_proximal_joint",
)

OPEN_POSITIONS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# Closed posture used by the hand mapping helper, in radians.
CLOSED_POSITIONS = (0.0, 0.6, 1.7, 1.7, 1.7, 1.7)

FINGER_LANDMARKS: Mapping[str, Tuple[int, int, int, int]] = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

PRESETS_0_TO_100 = {
    "open": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "close": (100.0, 100.0, 100.0, 100.0, 100.0, 100.0),
    "stop_grasp": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "prepare_grasp": (100.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "start_grasp": (40.0, 20.0, 80.0, 80.0, 80.0, 80.0),
    "default_grasp": (10.0, 30.0, 100.0, 100.0, 100.0, 100.0),
}


@dataclass(frozen=True)
class XYZ:
    """Small point type used by tests and conversion helpers."""

    x: float
    y: float
    z: float = 0.0


def normalize_hand_selection(hand: str) -> Tuple[str, ...]:
    """Return selected hand sides from a launch/parameter string."""

    value = str(hand).lower()
    if value == "both":
        return SIDES
    if value in SIDES:
        return (value,)
    raise ValueError("hand must be one of: left, right, both")


def joint_names_for_side(side: str) -> Tuple[str, ...]:
    """Return controller joint names for a robot hand side."""

    value = str(side).lower()
    if value == "right":
        return RIGHT_JOINTS
    if value == "left":
        return LEFT_JOINTS
    raise ValueError("side must be one of: left, right")


def normalize_handedness(handedness: str, mirror_handedness: bool = False) -> str:
    """Normalize MediaPipe handedness labels to robot side names."""

    label = str(handedness or "").strip().lower()
    if label.startswith("right"):
        side = "right"
    elif label.startswith("left"):
        side = "left"
    else:
        side = ""

    if mirror_handedness and side == "right":
        return "left"
    if mirror_handedness and side == "left":
        return "right"
    return side


def clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    """Clamp a numeric value, treating non-finite values as the lower bound."""

    if not math.isfinite(value):
        return lower
    return min(upper, max(lower, value))


def point_xyz(point) -> XYZ:
    """Convert a geometry-like point into an XYZ tuple."""

    return XYZ(float(point.x), float(point.y), float(getattr(point, "z", 0.0)))


def _sub(a: XYZ, b: XYZ) -> XYZ:
    return XYZ(a.x - b.x, a.y - b.y, a.z - b.z)


def _norm(v: XYZ) -> float:
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _distance(a: XYZ, b: XYZ) -> float:
    return _norm(_sub(a, b))


def _angle_deg(a: XYZ, b: XYZ, c: XYZ) -> float:
    ba = _sub(a, b)
    bc = _sub(c, b)
    denom = _norm(ba) * _norm(bc)
    if denom < 1e-9:
        return 180.0
    cos_value = clamp(
        (ba.x * bc.x + ba.y * bc.y + ba.z * bc.z) / denom,
        -1.0,
        1.0,
    )
    return math.degrees(math.acos(cos_value))


def _closure_from_angle(
    angle_deg: float,
    open_angle_deg: float = 165.0,
    closed_angle_deg: float = 75.0,
) -> float:
    return clamp((open_angle_deg - angle_deg) / (open_angle_deg - closed_angle_deg))


def _closure_from_distance_ratio(
    ratio: float,
    open_ratio: float,
    closed_ratio: float,
) -> float:
    return clamp((open_ratio - ratio) / (open_ratio - closed_ratio))


def _closure_from_value(
    value: float,
    open_value: float,
    closed_value: float,
) -> float:
    return clamp((value - open_value) / (closed_value - open_value))


def _chain_length(points: Sequence[XYZ], landmark_ids: Sequence[int]) -> float:
    return sum(
        _distance(points[start], points[end])
        for start, end in zip(landmark_ids, landmark_ids[1:])
    )


def _closure_from_finger_chord(
    points: Sequence[XYZ],
    landmark_ids: Sequence[int],
    open_ratio: float = 0.96,
    closed_ratio: float = 0.55,
) -> float:
    chain = _chain_length(points, landmark_ids)
    if chain < 1e-9:
        return 0.0
    chord_ratio = _distance(points[landmark_ids[0]], points[landmark_ids[-1]]) / chain
    return _closure_from_distance_ratio(chord_ratio, open_ratio, closed_ratio)


def _finger_closure(points: Sequence[XYZ], finger: str) -> float:
    landmark_ids = FINGER_LANDMARKS[finger]
    if finger == "thumb":
        cmc, mcp, ip, tip = landmark_ids
        mcp_closure = _closure_from_angle(
            _angle_deg(points[cmc], points[mcp], points[ip]),
            open_angle_deg=165.0,
            closed_angle_deg=90.0,
        )
        ip_closure = _closure_from_angle(
            _angle_deg(points[mcp], points[ip], points[tip]),
            open_angle_deg=165.0,
            closed_angle_deg=80.0,
        )
        chord_closure = _closure_from_finger_chord(
            points,
            landmark_ids,
            open_ratio=0.96,
            closed_ratio=0.60,
        )
        return max(mcp_closure, ip_closure, chord_closure)

    mcp, pip, dip, tip = landmark_ids
    pip_closure = _closure_from_angle(
        _angle_deg(points[mcp], points[pip], points[dip]),
        open_angle_deg=170.0,
        closed_angle_deg=80.0,
    )
    dip_closure = _closure_from_angle(
        _angle_deg(points[pip], points[dip], points[tip]),
        open_angle_deg=170.0,
        closed_angle_deg=100.0,
    )
    chord_closure = _closure_from_finger_chord(
        points,
        landmark_ids,
        open_ratio=0.96,
        closed_ratio=0.55,
    )
    return max(pip_closure, dip_closure, chord_closure)


def _thumb_opposition(points: Sequence[XYZ]) -> float:
    palm_width = max(_distance(points[5], points[17]), 1e-6)
    thumb_to_index = _distance(points[4], points[5]) / palm_width
    return _closure_from_distance_ratio(
        thumb_to_index,
        open_ratio=1.05,
        closed_ratio=0.45,
    )


def _thumb_folded_with_closed_fingers(
    points: Sequence[XYZ],
    finger_closures: Sequence[float],
) -> float:
    finger_gate = _closure_from_value(
        sum(finger_closures) / len(finger_closures),
        open_value=0.55,
        closed_value=0.85,
    )
    palm_width = max(_distance(points[5], points[17]), 1e-6)
    thumb_tip_to_fingers = min(
        _distance(points[4], points[landmark_id]) / palm_width
        for landmark_id in range(5, 21)
    )
    proximity = _closure_from_distance_ratio(
        thumb_tip_to_fingers,
        open_ratio=1.05,
        closed_ratio=0.45,
    )
    return finger_gate * proximity


def landmarks_to_closure_fractions(landmarks: Sequence[object]) -> Tuple[float, ...]:
    """Convert 21 MediaPipe hand landmarks to six 0=open, 1=closed fractions."""

    if len(landmarks) < 21:
        raise ValueError(f"expected at least 21 hand landmarks, got {len(landmarks)}")

    points = [point_xyz(point) for point in landmarks[:21]]
    finger_closures = (
        _finger_closure(points, "index"),
        _finger_closure(points, "middle"),
        _finger_closure(points, "ring"),
        _finger_closure(points, "pinky"),
    )
    thumb_fist_closure = _thumb_folded_with_closed_fingers(points, finger_closures)
    thumb_pitch = max(_finger_closure(points, "thumb"), thumb_fist_closure)
    thumb_yaw = max(
        clamp(0.7 * _thumb_opposition(points) + 0.3 * thumb_pitch),
        thumb_fist_closure,
    )
    return (
        thumb_yaw,
        thumb_pitch,
        *finger_closures,
    )


def fractions_to_positions(fractions: Sequence[float]) -> List[float]:
    """Map six closure fractions into Inspire joint positions in radians."""

    if len(fractions) != 6:
        raise ValueError(f"expected six closure fractions, got {len(fractions)}")
    return [
        open_pos + clamp(frac) * (closed_pos - open_pos)
        for frac, open_pos, closed_pos in zip(
            fractions, OPEN_POSITIONS, CLOSED_POSITIONS
        )
    ]


def landmarks_to_positions(landmarks: Sequence[object]) -> List[float]:
    """Convert 21 MediaPipe landmarks directly to Inspire joint positions."""

    return fractions_to_positions(landmarks_to_closure_fractions(landmarks))


def percentages_to_fractions(values: Iterable[float]) -> Tuple[float, ...]:
    """Convert one or six user 0..100 values into six closure fractions."""

    raw = [float(value) for value in values]
    if len(raw) == 1:
        raw = raw * 6
    if len(raw) != 6:
        raise ValueError("expected either one value or six hand values")
    return tuple(clamp(value / 100.0) for value in raw)


def percentages_to_positions(values: Iterable[float]) -> List[float]:
    """Convert user 0..100 hand command values to Inspire joint positions."""

    return fractions_to_positions(percentages_to_fractions(values))


def preset_percentages(name: str) -> Tuple[float, ...]:
    """Return a named 0..100 command vector."""

    key = str(name).lower().replace("-", "_")
    if key not in PRESETS_0_TO_100:
        raise ValueError(
            "unknown preset '%s'; expected one of: %s"
            % (name, ", ".join(sorted(PRESETS_0_TO_100)))
        )
    return PRESETS_0_TO_100[key]


def smooth_positions(
    previous: Sequence[float] | None,
    current: Sequence[float],
    alpha: float,
) -> List[float]:
    """Apply first-order smoothing to a command vector."""

    if previous is None:
        return [float(value) for value in current]
    weight = clamp(alpha)
    return [
        (1.0 - weight) * float(old) + weight * float(new)
        for old, new in zip(previous, current)
    ]
