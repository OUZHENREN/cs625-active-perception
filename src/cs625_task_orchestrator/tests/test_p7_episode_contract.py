from cs625_task_orchestrator.p7_episode_contract import SCHEMA_VERSION, evaluate_episode


def _evidence():
    return {
        "perception": {
            "fresh_rgbd": True,
            "visible_ratio": 0.4,
            "translation_error_m": 0.01,
            "rotation_error_deg": 4.0,
        },
        "pregrasp": {"plan_success": True, "execution_success": True},
        "approach": {"plan_success": True, "execution_success": True},
        "gripper": {
            "command_reached": True,
            "geometry_measurement_available": True,
            "target_to_grasp_center_error_m": 0.01,
            "geometry_pass": True,
        },
        "attachment": {"attached": True, "relative_translation_error_m": 0.01},
        "lift": {"plan_success": True, "execution_success": True, "height_delta_m": 0.12},
        "hold": {"duration_sec": 2.1, "stable": True},
        "contact": {"monitor_available": True, "unexpected_collision": False},
    }


def test_p7_success_requires_every_stage_and_contact_evidence():
    result = evaluate_episode(_evidence())
    assert result["metric_schema_version"] == SCHEMA_VERSION
    assert result["perception_success"]
    assert result["executable_grasp"]
    assert result["attachment_success"]
    assert result["task_success"]
    assert result["physical_collision_metric_accepted"]
    assert result["unexpected_physical_collision"] is False


def test_p7_missing_pose_evidence_never_defaults_to_perception_success():
    evidence = _evidence()
    del evidence["perception"]["translation_error_m"]
    result = evaluate_episode(evidence)
    assert not result["perception_success"]
    assert result["primary_failure_code"] == "POSE_TRANSLATION_ERROR"


def test_p7_contact_is_not_imputed_from_p3_or_attachment():
    evidence = _evidence()
    evidence["contact"] = {}
    result = evaluate_episode(evidence)
    assert not result["task_success"]
    assert result["primary_failure_code"] == "PHYSICAL_COLLISION_UNOBSERVED"
    assert not result["physical_collision_metric_accepted"]
    assert result["unexpected_physical_collision"] is None


def test_p7_contact_monitor_requires_an_explicit_boolean_observation():
    evidence = _evidence()
    evidence["contact"]["unexpected_collision"] = None
    result = evaluate_episode(evidence)
    assert not result["task_success"]
    assert result["primary_failure_code"] == "PHYSICAL_COLLISION_UNOBSERVED"
    assert not result["physical_collision_metric_accepted"]


def test_p7_non_finite_measurements_never_pass_thresholds():
    evidence = _evidence()
    evidence["lift"]["height_delta_m"] = float("nan")
    result = evaluate_episode(evidence)
    assert not result["task_success"]
    assert result["primary_failure_code"] == "OBJECT_NOT_LIFTED"


def test_p7_measured_grasp_geometry_is_not_imputed_from_motion_success():
    evidence = _evidence()
    del evidence["gripper"]["geometry_measurement_available"]
    result = evaluate_episode(evidence)
    assert not result["executable_grasp"]
    assert result["primary_failure_code"] == "GRASP_GEOMETRY_UNOBSERVED"


def test_p7_unexpected_contact_is_a_task_failure_when_monitored():
    evidence = _evidence()
    evidence["contact"]["unexpected_collision"] = True
    result = evaluate_episode(evidence)
    assert not result["task_success"]
    assert result["primary_failure_code"] == "UNEXPECTED_COLLISION"
