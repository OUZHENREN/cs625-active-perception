"""P4 决策逻辑的单元测试。"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from cs625_task_orchestrator.p4_decision_logic import (  # noqa: E402
    Action,
    DecisionState,
    FailureCode,
)


def test_timeout_retry_then_plan_success():
    state = DecisionState(max_retries=2)

    assert state.on_timeout() is Action.RETRY
    assert state.on_pose_valid() is Action.PLAN
    assert state.on_plan_ok() is Action.SUCCESS

    result = state.result()
    assert result["decision"] == "SUCCESS"
    assert result["attempts"] == 1
    assert result["retries"] == 1
    assert result["planning_calls"] == 0  # 由 ROS 节点负责计数；纯逻辑不重复记
    assert result["failures"] == ["VISION_NO_MATCH"]
    assert [item["event"] for item in result["history"]] == [
        "VISION_NO_MATCH",
        "POSE_VALID",
        "PLAN_OK",
    ]


def test_ng_status_maps_to_vision_no_match_without_timeout():
    state = DecisionState(max_retries=1)

    assert state.on_vision_no_match() is Action.RETRY
    assert state.on_vision_no_match() is Action.ABORT

    result = state.result()
    assert result["decision"] == "ABORT"
    assert result["retries"] == 1
    assert result["failures"] == ["VISION_NO_MATCH", "VISION_NO_MATCH"]


def test_plan_failed_retries_once_then_success():
    state = DecisionState(max_retries=2)

    assert state.on_pose_valid() is Action.PLAN
    assert state.on_plan_failed() is Action.RETRY
    assert state.retries == 1
    assert state.on_pose_valid() is Action.PLAN
    assert state.on_plan_ok() is Action.SUCCESS

    result = state.result()
    assert result["decision"] == "SUCCESS"
    assert result["failures"] == ["PLAN_FAILED"]
    assert result["attempts"] == 2


def test_retry_exhausted_after_max_retries():
    state = DecisionState(max_retries=2)

    assert state.on_timeout() is Action.RETRY
    assert state.on_timeout() is Action.RETRY
    assert state.on_timeout() is Action.ABORT

    result = state.result()
    assert result["decision"] == "ABORT"
    assert result["retries"] == 2
    assert result["failures"] == [
        "VISION_NO_MATCH",
        "VISION_NO_MATCH",
        "VISION_NO_MATCH",
    ]


def test_timeout_and_ng_share_the_same_failure_code():
    state = DecisionState(max_retries=5)

    state.on_timeout()
    state.on_vision_no_match()

    assert state.failures == [
        FailureCode.VISION_NO_MATCH,
        FailureCode.VISION_NO_MATCH,
    ]
