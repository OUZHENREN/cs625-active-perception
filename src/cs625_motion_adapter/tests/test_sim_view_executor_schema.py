from types import SimpleNamespace

from cs625_motion_adapter.sim_view_executor import execution_status_payload


def test_execution_status_payload_preserves_runtime_phase_metrics():
    selection = SimpleNamespace(
        cycle_index=2,
        candidate=SimpleNamespace(
            candidate_id="fibonacci_08",
            motion_cost=1.25,
            planning_time_sec=0.04,
        ),
    )

    payload = execution_status_payload(
        selection,
        True,
        "SENSOR_SETTLED",
        "sim",
        10.0,
        12.5,
        {
            "ik_wall_time_sec": 0.10,
            "plan_wall_time_sec": 0.20,
            "result_wall_time_sec": 1.75,
            "waiting_sensor_wall_time_sec": 0.05,
        },
    )

    assert payload["view_wall_time_sec"] == 2.5
    assert payload["execution_ik_time_sec"] == 0.10
    assert payload["execution_motion_planning_time_sec"] == 0.20
    assert payload["trajectory_execution_time_sec"] == 1.75
    assert payload["sensor_settle_wait_time_sec"] == 0.05
