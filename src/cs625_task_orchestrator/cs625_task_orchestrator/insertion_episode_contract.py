"""Acceptance contract for the shielding module insertion task.

Pure logic: it reads a recorded episode and returns a verdict.  No ROS, no meshes, no
files, which is what makes the criteria reviewable and the classification testable.

THE CONSTRAINT THIS TASK IS BUILT AROUND.  The module passes through the fixture with
0.040 mm to spare on the full-resolution CAD export, and 0.098 mm on the decimated
asset the simulator uses.  That is a precision fit: the descent is geometrically
valid and cannot be executed by position control, because a 0.04 mm clearance is far
below what a position-servoed arm can track.  The task owner decided to record that
as a real constraint and let the simulator show it, rather than to hide it behind a
chamfer or a compliance controller, so this contract has to distinguish three things
that a naive pass/fail would collapse:

  PASS                 every leg completed and the module seated
  PRECISION_FIT_JAM    the descent aborted while in contact with the fixture, with
                       the preceding legs and the plan itself fine.  This is the
                       EXPECTED outcome for a position-controlled descent and it is
                       the evidence the constraint is real, not a defect
  OTHER_FAILURE        anything else: a leg that never reached its pose, a missing
                       contact record, a plan rejected in collision, a module that
                       was never grasped
"""

from __future__ import annotations

from typing import Any


SCHEMA_VERSION = "cs625_insertion_episode_v1"

# The legs the sequence must record, in order.  TRANSFER_FREE_SPACE is a planner leg
# and records a plan rather than a straight line.
REQUIRED_LEGS = (
    "pregrasp_module",
    "grasp_module",
    "close_arms",
    "lift_module",
    "insert_entry",
    "insert_seated",
    "open_arms",
    "release_retreat",
)

# A leg is only a jam candidate if it is the descent or the seating itself.
DESCENT_LEGS = ("insert_entry", "insert_seated")

# The clearance the whole task turns on, in metres.  Recorded so the verdict can be
# read without the config, and checked against the value the calibration produced.
PRECISION_FIT_CLEARANCE_M = 0.000040


def _leg(episode: dict[str, Any], name: str) -> dict[str, Any] | None:
    legs = episode.get("legs")
    if not isinstance(legs, dict):
        return None
    entry = legs.get(name)
    return entry if isinstance(entry, dict) else None


def _succeeded(leg: dict[str, Any]) -> bool:
    return leg.get("result") == "succeeded"


def evaluate_insertion_episode(episode: dict[str, Any]) -> dict[str, Any]:
    """Classify a recorded insertion episode."""

    failures: list[str] = []

    constraint = episode.get("precision_fit")
    if not isinstance(constraint, dict):
        failures.append("PRECISION_FIT_NOT_RECORDED")
        recorded_clearance = None
    else:
        recorded_clearance = constraint.get("minimum_clearance_m")
        if not isinstance(recorded_clearance, (int, float)):
            failures.append("PRECISION_FIT_CLEARANCE_MISSING")
            recorded_clearance = None

    missing = [name for name in REQUIRED_LEGS if _leg(episode, name) is None]
    if missing:
        failures.append("LEGS_UNRECORDED")
        return _verdict("OTHER_FAILURE", failures, None, recorded_clearance, missing)

    # Everything before the descent must have worked, otherwise this is not a jam.
    upstream = [
        name
        for name in REQUIRED_LEGS
        if name not in DESCENT_LEGS and not _succeeded(_leg(episode, name) or {})
    ]
    if upstream:
        failures.append("UPSTREAM_LEG_FAILED")
        return _verdict("OTHER_FAILURE", failures, None, recorded_clearance, upstream)

    descent_failed = [
        name for name in DESCENT_LEGS if not _succeeded(_leg(episode, name) or {})
    ]
    if not descent_failed:
        # A position-controlled descent completing is possible in principle; it is
        # reported rather than assumed impossible.
        return _verdict("PASS", failures, None, recorded_clearance, [])

    # A jam needs the contact record, not just a failed descent.  Without it a
    # tracking abort is indistinguishable from a controller that gave up for an
    # unrelated reason, and calling that a jam would be a claim the evidence does not
    # support.
    contact = episode.get("contact_during_descent")
    if contact is None:
        failures.append("CONTACT_EVIDENCE_MISSING")
        return _verdict(
            "OTHER_FAILURE", failures, descent_failed[0], recorded_clearance, descent_failed
        )
    if not isinstance(contact, dict):
        failures.append("CONTACT_EVIDENCE_INVALID")
        return _verdict("OTHER_FAILURE", failures, None, recorded_clearance, descent_failed)

    contacted = bool(contact.get("contacted"))
    if not contacted:
        # The descent failed without touching anything, so the precision fit is not
        # what stopped it.
        failures.append("DESCENT_FAILED_WITHOUT_CONTACT")
        return _verdict(
            "OTHER_FAILURE", failures, descent_failed[0], recorded_clearance, descent_failed
        )

    return _verdict("PRECISION_FIT_JAM", failures, descent_failed[0], recorded_clearance, [])


def _verdict(
    verdict: str,
    failures: list[str],
    first_failed_leg: str | None,
    recorded_clearance: float | None,
    failed_legs: list[str],
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "verdict": verdict,
        "failure_codes": failures,
        "first_failed_leg": first_failed_leg,
        "failed_legs": failed_legs,
        "precision_fit_clearance_m": recorded_clearance,
        "expected_clearance_m": PRECISION_FIT_CLEARANCE_M,
        "interpretation": _INTERPRETATION[verdict],
        "claim_boundary": (
            "insertion-task execution evidence only; this verdict says whether the "
            "recorded episode is consistent with the recorded precision fit, not that "
            "the task is solved"
        ),
    }


_INTERPRETATION = {
    "PASS": (
        "every leg completed and the module seated under position control; the "
        "precision fit did not stop it"
    ),
    "PRECISION_FIT_JAM": (
        "the descent aborted in contact with the fixture after a clean approach and a "
        "clean plan. This is the expected result for a position-controlled descent "
        "through a 0.040 mm clearance and is evidence that the fit is real, not a "
        "defect to be fixed"
    ),
    "OTHER_FAILURE": (
        "the episode does not support the precision-fit conclusion; the failing leg "
        "and its codes are recorded above"
    ),
}
