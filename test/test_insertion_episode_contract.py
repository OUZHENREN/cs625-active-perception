"""Regression tests for the insertion episode verdict.

The task owner chose to record the 0.040 mm fit as a real constraint and let the
simulator show it, so the contract's job is to tell a precision-fit jam apart from a
generic failure.  Collapsing those two is the failure mode these tests guard: it
would let the conclusion be claimed from an episode that does not support it.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "cs625_task_orchestrator"))
from cs625_task_orchestrator import insertion_episode_contract as contract  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    "assemble_insertion_episode", ROOT / "test" / "assemble_insertion_episode.py"
)
assembler = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(assembler)


def episode(*, failed=(), contacted=True, clearance=0.000040, omit_contact=False):
    legs = {
        name: {"result": "failed" if name in failed else "succeeded", "code": ""}
        for name in contract.REQUIRED_LEGS
    }
    record = {
        "precision_fit": {"minimum_clearance_m": clearance},
        "legs": legs,
        "order": list(contract.REQUIRED_LEGS),
    }
    if not omit_contact:
        record["contact_during_descent"] = {"contacted": contacted, "contact_count": 3}
    return record


def test_a_clean_descent_is_a_pass():
    verdict = contract.evaluate_insertion_episode(episode())
    assert verdict["verdict"] == "PASS"
    assert verdict["failure_codes"] == []


def test_a_descent_failing_in_contact_is_the_expected_jam():
    for leg in contract.DESCENT_LEGS:
        verdict = contract.evaluate_insertion_episode(episode(failed=(leg,)))
        assert verdict["verdict"] == "PRECISION_FIT_JAM"
        assert verdict["first_failed_leg"] == leg
        assert verdict["failure_codes"] == []
        assert "0.040 mm" in verdict["interpretation"]


def test_a_descent_failing_without_contact_is_not_a_jam():
    """The distinction the whole task turns on.

    A tracking abort with nothing touching anything is a controller problem.  Calling
    it a precision-fit jam would let the conclusion be claimed from an episode that
    does not demonstrate it.
    """

    verdict = contract.evaluate_insertion_episode(
        episode(failed=("insert_seated",), contacted=False)
    )
    assert verdict["verdict"] == "OTHER_FAILURE"
    assert "DESCENT_FAILED_WITHOUT_CONTACT" in verdict["failure_codes"]


def test_a_missing_contact_record_is_not_a_jam():
    verdict = contract.evaluate_insertion_episode(
        episode(failed=("insert_seated",), omit_contact=True)
    )
    assert verdict["verdict"] == "OTHER_FAILURE"
    assert "CONTACT_EVIDENCE_MISSING" in verdict["failure_codes"]


def test_a_failure_before_the_descent_is_not_a_jam():
    verdict = contract.evaluate_insertion_episode(episode(failed=("lift_module",)))
    assert verdict["verdict"] == "OTHER_FAILURE"
    assert "UPSTREAM_LEG_FAILED" in verdict["failure_codes"]


def test_an_unrecorded_leg_is_not_a_jam():
    record = episode(failed=("insert_seated",))
    del record["legs"]["open_arms"]
    verdict = contract.evaluate_insertion_episode(record)
    assert verdict["verdict"] == "OTHER_FAILURE"
    assert "LEGS_UNRECORDED" in verdict["failure_codes"]


def test_a_missing_precision_fit_record_is_flagged():
    record = episode()
    del record["precision_fit"]
    verdict = contract.evaluate_insertion_episode(record)
    assert "PRECISION_FIT_NOT_RECORDED" in verdict["failure_codes"]


def test_assembler_reads_receipts_and_requires_contacts(tmp_path):
    leg_dir = tmp_path / "legs"
    leg_dir.mkdir()
    for name in contract.REQUIRED_LEGS:
        failed = name == "insert_seated"
        (leg_dir / f"{name}.json").write_text(
            json.dumps(
                {
                    "capture_schema": "p7_phase_receipt_v1",
                    "captured_at_utc": "2026-09-21T00:00:00Z",
                    "receipt": {"success": not failed, "code": "ABORTED" if failed else ""},
                }
            ),
            encoding="utf-8",
        )
    contacts = tmp_path / "contacts.json"
    contacts.write_text(
        json.dumps(
            {
                "schema": "p7_gazebo_contact_capture_v1",
                "contacts": [{"collision1": {"name": "envelope"}}],
                "topics": ["/cs625_task/contacts/module"],
            }
        ),
        encoding="utf-8",
    )
    sequence = ROOT / "src/cs625_bringup/config/cs625_insertion_sequence.yaml"
    record = assembler.assemble(leg_dir, contacts, sequence)
    assert record["legs"]["insert_seated"]["result"] == "failed"
    assert record["legs"]["lift_module"]["result"] == "succeeded"
    assert record["contact_during_descent"]["contacted"] is True
    assert record["precision_fit"]["minimum_clearance_m"] == pytest.approx(0.000040)
    assert contract.evaluate_insertion_episode(record)["verdict"] == "PRECISION_FIT_JAM"


def test_assembler_records_a_leg_that_never_ran(tmp_path):
    leg_dir = tmp_path / "legs"
    leg_dir.mkdir()
    contacts = tmp_path / "contacts.json"
    contacts.write_text(json.dumps({"contacts": []}), encoding="utf-8")
    sequence = ROOT / "src/cs625_bringup/config/cs625_insertion_sequence.yaml"
    record = assembler.assemble(leg_dir, contacts, sequence)
    assert record["legs"]["pregrasp_module"]["result"] == "unrecorded"
    assert record["legs"]["pregrasp_module"]["code"] == "LEG_NOT_RUN"
    assert contract.evaluate_insertion_episode(record)["verdict"] == "OTHER_FAILURE"
