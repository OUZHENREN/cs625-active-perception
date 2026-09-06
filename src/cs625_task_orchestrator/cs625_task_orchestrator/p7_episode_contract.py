"""Auditable P7 grasp episode acceptance rules.

This module deliberately consumes recorded evidence instead of fabricating a
task outcome from P4 motion or point-cloud status.  It is reusable both by the
ROS task orchestrator and by offline result validation.
"""

from __future__ import annotations

import math
from typing import Any


SCHEMA_VERSION = "p7_grasp_evidence_v2"


def _flag(section: dict[str, Any], key: str) -> bool:
    return bool(section.get(key, False))


def _number(section: dict[str, Any], key: str) -> float | None:
    value = section.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _first_failure(*candidates: tuple[bool, str]) -> str | None:
    for condition, code in candidates:
        if condition:
            return code
    return None


def evaluate_episode(evidence: dict[str, Any]) -> dict[str, Any]:
    """Evaluate evidence for one P7 episode without assuming unrecorded facts.

    Required sections are ``perception``, ``pregrasp``, ``approach``,
    ``gripper``, ``attachment``, ``lift`` and ``hold``.  Missing evidence is a
    failure, never a default success.  Physical collision is *accepted* only
    when a contact monitor explicitly provided an observation; P3 candidate
    collision filtering is intentionally not consumed here.
    """

    perception = dict(evidence.get("perception") or {})
    pregrasp = dict(evidence.get("pregrasp") or {})
    approach = dict(evidence.get("approach") or {})
    gripper = dict(evidence.get("gripper") or {})
    attachment = dict(evidence.get("attachment") or {})
    lift = dict(evidence.get("lift") or {})
    hold = dict(evidence.get("hold") or {})
    contact = dict(evidence.get("contact") or {})

    translation_error = _number(perception, "translation_error_m")
    rotation_error = _number(perception, "rotation_error_deg")
    visible_ratio = _number(perception, "visible_ratio")
    perception_failure = _first_failure(
        (not _flag(perception, "fresh_rgbd"), "PERCEPTION_STALE"),
        (visible_ratio is None or visible_ratio < 0.15, "LOW_VISIBILITY"),
        (translation_error is None or translation_error > 0.02, "POSE_TRANSLATION_ERROR"),
        (rotation_error is None or rotation_error > 10.0, "POSE_ROTATION_ERROR"),
    )
    perception_success = perception_failure is None

    executable_failure = _first_failure(
        (not perception_success, perception_failure or "PERCEPTION_NOT_ACCEPTED"),
        (not _flag(pregrasp, "plan_success"), "PREGRASP_PLAN_FAILED"),
        (not _flag(pregrasp, "execution_success"), "PREGRASP_EXECUTION_FAILED"),
        (not _flag(approach, "plan_success"), "APPROACH_PLAN_FAILED"),
        (not _flag(approach, "execution_success"), "APPROACH_EXECUTION_FAILED"),
        (not _flag(gripper, "command_reached"), "GRIPPER_TIMEOUT"),
        (not _flag(gripper, "geometry_measurement_available"), "GRASP_GEOMETRY_UNOBSERVED"),
        (
            _number(gripper, "target_to_grasp_center_error_m") is None
            or _number(gripper, "target_to_grasp_center_error_m") > 0.015,
            "GRASP_GEOMETRY_ERROR",
        ),
        (not _flag(gripper, "geometry_pass"), "GRASP_GEOMETRY_REJECTED"),
    )
    executable_grasp = executable_failure is None

    relative_error = _number(attachment, "relative_translation_error_m")
    attachment_failure = _first_failure(
        (not executable_grasp, executable_failure or "GRASP_NOT_EXECUTABLE"),
        (not _flag(attachment, "attached"), "ATTACHMENT_FAILED"),
        (relative_error is None or relative_error > 0.015, "ATTACHMENT_POSE_ERROR"),
    )
    attachment_success = attachment_failure is None

    height_delta = _number(lift, "height_delta_m")
    hold_duration = _number(hold, "duration_sec")
    physical_collision_observed = contact.get("unexpected_collision")
    contact_monitor_available = _flag(contact, "monitor_available")
    collision_observation_available = (
        contact_monitor_available
        and isinstance(physical_collision_observed, bool)
    )
    task_failure = _first_failure(
        (not attachment_success, attachment_failure or "ATTACHMENT_NOT_ACCEPTED"),
        (not _flag(lift, "plan_success"), "LIFT_PLAN_FAILED"),
        (not _flag(lift, "execution_success"), "LIFT_EXECUTION_FAILED"),
        (height_delta is None or height_delta < 0.10, "OBJECT_NOT_LIFTED"),
        (hold_duration is None or hold_duration < 2.0, "HOLD_FAILED"),
        (not _flag(hold, "stable"), "HOLD_UNSTABLE"),
        (not collision_observation_available, "PHYSICAL_COLLISION_UNOBSERVED"),
        (physical_collision_observed is True, "UNEXPECTED_COLLISION"),
    )
    task_success = task_failure is None

    return {
        "metric_schema_version": SCHEMA_VERSION,
        "perception_success": perception_success,
        "executable_grasp": executable_grasp,
        "attachment_success": attachment_success,
        "task_success": task_success,
        "primary_failure_code": task_failure or "TASK_SUCCEEDED",
        "physical_collision_metric_accepted": collision_observation_available,
        "unexpected_physical_collision": (
            physical_collision_observed if collision_observation_available else None
        ),
        "thresholds": {
            "visible_ratio_min": 0.15,
            "translation_error_max_m": 0.02,
            "rotation_error_max_deg": 10.0,
            "attachment_relative_translation_max_m": 0.015,
            "target_to_grasp_center_error_max_m": 0.015,
            "lift_height_min_m": 0.10,
            "hold_duration_min_sec": 2.0,
        },
    }
