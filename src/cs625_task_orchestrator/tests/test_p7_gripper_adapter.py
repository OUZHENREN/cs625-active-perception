import pytest

from cs625_task_orchestrator.p7_gripper_adapter import gripper_status_payload


def test_gripper_status_payload_is_explicit_about_command_result():
    assert gripper_status_payload("close-1", True, "GRIPPER_REACHED", 0.01, 0.25) == {
        "command_id": "close-1",
        "success": True,
        "code": "GRIPPER_REACHED",
        "target_position": 0.01,
        "elapsed_sec": 0.25,
    }


def test_gripper_status_payload_preserves_both_measured_fingers():
    payload = gripper_status_payload(
        "close-2",
        True,
        "GRIPPER_REACHED",
        0.023,
        0.5,
        {
            "gripper_left_finger_joint": 0.0228,
            "gripper_right_finger_joint": 0.0231,
        },
        0.002,
    )
    assert payload["max_terminal_error_m"] == pytest.approx(0.0002)
    assert payload["target_tolerance_m"] == 0.002
    assert set(payload["observed_positions_m"]) == {
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    }
