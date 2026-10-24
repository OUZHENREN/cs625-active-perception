"""P4 最小决策逻辑：失败码 + 有界重试。

这一层不接触 ROS，只负责“收到什么事件、下一步做什么、最终结论是什么”，
便于用单元测试钉住行为；ROS 节点 `p4_decision_demo.py` 只做 IO。

事件语义：
- POSE_VALID     : 收到一条可规划的目标位姿（例如 /box_pre_grasp_tcp_pose）
- VISION_NO_MATCH: RVS 明确回报 NG（/rvs/pose_status），或等待位姿超时
- PLAN_OK        : 规划服务返回 success=True
- PLAN_FAILED    : 规划服务返回 success=False

重试语义：每次失败（视觉没匹配 / 规划失败）消耗一次重试机会；
`max_retries=2` 表示最多再重新观测 2 次，总共最多 3 次尝试。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class FailureCode(str, Enum):
    VISION_NO_MATCH = "VISION_NO_MATCH"
    PLAN_FAILED = "PLAN_FAILED"
    RETRY_EXHAUSTED = "RETRY_EXHAUSTED"


class Action(str, Enum):
    PLAN = "PLAN"
    RETRY = "RETRY"
    SUCCESS = "SUCCESS"
    ABORT = "ABORT"


@dataclass
class DecisionState:
    max_retries: int = 2
    retries: int = 0
    attempts: int = 0
    planning_calls: int = 0
    final: str | None = None
    failures: list[FailureCode] = field(default_factory=list)
    history: list[dict[str, Any]] = field(default_factory=list)

    def _record(self, event: str, action: Action, code: FailureCode | None = None) -> None:
        self.history.append(
            {
                "event": event,
                "action": action.value,
                "failure": code.value if code else None,
                "attempts": self.attempts,
                "retries": self.retries,
            }
        )

    def _retry_or_abort(self, code: FailureCode) -> Action:
        self.failures.append(code)
        if self.retries < self.max_retries:
            self.retries += 1
            self._record(code.value, Action.RETRY, code)
            return Action.RETRY
        self.final = "ABORT"
        self._record("RETRY_EXHAUSTED", Action.ABORT, FailureCode.RETRY_EXHAUSTED)
        return Action.ABORT

    def on_pose_valid(self) -> Action:
        self.attempts += 1
        self._record("POSE_VALID", Action.PLAN)
        return Action.PLAN

    def on_plan_ok(self) -> Action:
        self.final = "SUCCESS"
        self._record("PLAN_OK", Action.SUCCESS)
        return Action.SUCCESS

    def on_plan_failed(self) -> Action:
        return self._retry_or_abort(FailureCode.PLAN_FAILED)

    def on_vision_no_match(self) -> Action:
        return self._retry_or_abort(FailureCode.VISION_NO_MATCH)

    def on_timeout(self) -> Action:
        # 真实 RVS 发 NG 时当前被 tcp_server 丢弃，决策层只能看到“没有位姿”；
        # 因此超时也映射到同一个失败码，保证行为一致。
        return self._retry_or_abort(FailureCode.VISION_NO_MATCH)

    def result(self) -> dict[str, Any]:
        return {
            "decision": self.final,
            "attempts": self.attempts,
            "retries": self.retries,
            "max_retries": self.max_retries,
            "planning_calls": self.planning_calls,
            "failures": [code.value for code in self.failures],
            "history": self.history,
        }
