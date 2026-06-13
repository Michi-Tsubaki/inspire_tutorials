"""Helpers for FollowJointTrajectory Inspire hand goals."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration_to_msg(duration_sec: float):
    """Return a builtin_interfaces/Duration-like value on a trajectory point."""

    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError(f"duration_sec must be positive, got {duration_sec}")
    point = JointTrajectoryPoint()
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int(
        round((duration_sec - point.time_from_start.sec) * 1e9)
    )
    if point.time_from_start.nanosec >= 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec -= 1_000_000_000
    return point.time_from_start


def make_trajectory_point(
    positions: Sequence[float],
    duration_sec: float,
) -> JointTrajectoryPoint:
    """Build a single-point joint trajectory point."""

    point = JointTrajectoryPoint()
    point.positions = [float(position) for position in positions]
    point.time_from_start = duration_to_msg(duration_sec)
    return point


def make_follow_joint_trajectory_goal(
    joint_names: Iterable[str],
    positions: Sequence[float],
    duration_sec: float,
) -> FollowJointTrajectory.Goal:
    """Build a FollowJointTrajectory goal for one Inspire hand."""

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    trajectory.points = [make_trajectory_point(positions, duration_sec)]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    return goal
