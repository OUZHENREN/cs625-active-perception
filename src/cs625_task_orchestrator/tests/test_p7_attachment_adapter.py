from cs625_task_orchestrator.p7_attachment_adapter import (
    attachment_status_payload,
    parse_gz_attachment_state,
)


def test_gz_attachment_state_parser_requires_explicit_evidence():
    assert parse_gz_attachment_state('data: "attached"') is True
    assert parse_gz_attachment_state('data: "detached"') is False
    assert parse_gz_attachment_state("") is None


def test_attachment_status_only_accepts_matching_explicit_state():
    attached = attachment_status_payload("a1", "attach", True, 0.2)
    detached = attachment_status_payload("a2", "detach", False, 0.2)
    unobserved = attachment_status_payload("a3", "attach", None, 0.2)
    assert attached["success"] is True and attached["code"] == "ATTACHMENT_ATTACHED"
    assert detached["success"] is True and detached["code"] == "ATTACHMENT_DETACHED"
    assert unobserved["success"] is False
    assert unobserved["code"] == "ATTACHMENT_STATE_UNOBSERVED"
