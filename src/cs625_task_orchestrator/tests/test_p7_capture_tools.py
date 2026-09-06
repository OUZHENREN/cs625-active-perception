from __future__ import annotations

import importlib.util
import math
import sys
from collections import deque
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def load_script(name: str):
    path = REPOSITORY_ROOT / "test" / name
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(f"p7_test_{path.stem}", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hold_window_excludes_first_query_latency_and_enforces_requested_duration():
    module = load_script("p7_capture_hold_trace.py")
    observed, drift, stable = module.evaluate_hold_window(
        [0.60, 1.18, 1.74, 2.88],
        [0.17095, 0.17095, 0.17095, 0.17095],
        2.20,
        0.01,
    )
    assert observed == pytest.approx(2.28)
    assert drift == 0.0
    assert stable

    _, _, too_short = module.evaluate_hold_window(
        [0.60, 2.70], [0.17095, 0.17095], 2.20, 0.01
    )
    assert not too_short


def test_hold_window_can_use_conservative_simulation_sample_bounds():
    module = load_script("p7_capture_hold_trace.py")
    samples = [
        {"query_started_sim_sec": 10.0, "query_finished_sim_sec": 10.6},
        {"query_started_sim_sec": 11.2, "query_finished_sim_sec": 11.8},
        {"query_started_sim_sec": 13.0, "query_finished_sim_sec": 13.6},
    ]
    assert module.conservative_elapsed_seconds(samples, "simulation") == pytest.approx(
        [0.0, 0.6, 2.4]
    )

    observed, drift, stable = module.evaluate_hold_window(
        module.conservative_elapsed_seconds(samples, "simulation"),
        [0.171, 0.171, 0.171],
        2.20,
        0.01,
    )
    assert observed == pytest.approx(2.4)
    assert drift == 0.0
    assert stable


def test_only_attached_mode_emits_an_attached_collision_object():
    module = load_script("p7_apply_fixture_scene.py")
    assert module.attached_objects_for_mode("full", None) == []
    assert module.attached_objects_for_mode("approach", None) == []

    relative_pose = {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    attached = module.attached_objects_for_mode("attached", relative_pose)
    assert len(attached) == 1
    assert attached[0].link_name == "p7_grasp_center_link"
    assert attached[0].object.id == "p7_target_contact_proxy"


def _stamped_message(message_type, stamp_sec: float, frame_id: str):
    message = message_type()
    whole = int(stamp_sec)
    message.header.stamp.sec = whole
    message.header.stamp.nanosec = int(round((stamp_sec - whole) * 1.0e9))
    message.header.frame_id = frame_id
    return message


def test_p7_1_synchronizer_requires_all_four_streams_within_frozen_slop():
    module = load_script("p7_capture_sensor_gate.py")
    buffers = {name: deque(maxlen=5) for name in module.STREAM_TYPES}
    buffers["color"].append(_stamped_message(module.Image, 10.00, "color"))
    buffers["depth"].append(_stamped_message(module.Image, 10.02, "depth"))
    buffers["camera_info"].append(
        _stamped_message(module.CameraInfo, 10.01, "color")
    )
    buffers["points"].append(
        _stamped_message(module.PointCloud2, 10.03, "depth")
    )
    selected = module.select_synchronized_window(
        buffers, after_points_stamp_sec=9.0, sync_slop_sec=0.05
    )
    assert selected is not None
    assert module.stamp_seconds(selected["points"]) == pytest.approx(10.03)
    assert (
        module.select_synchronized_window(
            buffers, after_points_stamp_sec=10.03, sync_slop_sec=0.05
        )
        is None
    )
    assert (
        module.select_synchronized_window(
            buffers, after_points_stamp_sec=9.0, sync_slop_sec=0.01
        )
        is None
    )


def test_p7_1_target_visibility_uses_geometry_not_pointcloud_receipt_proxy():
    module = load_script("p7_capture_sensor_gate.py")
    inside = module.TARGET_LOCAL_CENTER_M + module.np.array(
        [[0.034, 0.0, 0.0], [-0.034, 0.0, 0.01], [0.0, 0.034, -0.01]]
    )
    outside = module.np.array([[2.0, 2.0, 2.0], [-1.0, 0.0, 0.0]])
    result = module.target_visibility_from_points(
        module.np.vstack((inside, outside)),
        world_from_camera_rotation=module.np.eye(3),
        world_from_camera_translation=module.np.zeros(3),
        target_model_position_world_m=module.np.zeros(3),
        target_model_rotation_world=module.np.eye(3),
        radial_margin_m=0.0,
        axial_margin_m=0.0,
    )
    assert result["finite_point_count"] == 5
    assert result["target_point_count"] == 3


def test_p7_1_projection_and_numpy_scalar_records_are_json_serializable():
    module = load_script("p7_capture_sensor_gate.py")
    camera_info = module.CameraInfo()
    camera_info.width = 320
    camera_info.height = 240
    camera_info.k = [200.0, 0.0, 160.0, 0.0, 200.0, 120.0, 0.0, 0.0, 1.0]
    projection = module.project_target_center(
        target_center_world_m=module.np.array([0.0, 0.0, 1.0]),
        world_from_camera_rotation=module.np.eye(3),
        world_from_camera_translation=module.np.zeros(3),
        camera_info=camera_info,
    )
    assert projection["inside_image"] is True
    encoded = module.json.dumps(
        {"projection": projection, "numpy_boolean": module.np.bool_(True)},
        default=module.json_scalar,
    )
    assert '"numpy_boolean": true' in encoded


def test_p7_1_exact_time_tf_waits_for_matching_transform(monkeypatch):
    module = load_script("p7_capture_sensor_gate.py")

    class BufferStub:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, parent, child, requested_time):
            self.calls.append((parent, child, requested_time))
            if len(self.calls) == 1:
                raise module.TransformException("matching TF has not arrived yet")
            return "exact-transform"

    spins = []
    monkeypatch.setattr(
        module.rclpy,
        "spin_once",
        lambda node, timeout_sec: spins.append((node, timeout_sec)),
    )
    stamp = module.Image().header.stamp
    stamp.sec = 12
    stamp.nanosec = 345000000
    buffer = BufferStub()
    result = module.lookup_transform_at_exact_stamp(
        "node", buffer, "world", "camera", stamp, timeout_sec=0.1
    )
    assert result == "exact-transform"
    assert len(buffer.calls) == 2
    assert spins and 0.0 < spins[0][1] <= 0.02
    assert buffer.calls[0][2].nanoseconds == 12345000000


def test_p7_1_static_projection_uses_urdf_chain_and_optical_z(tmp_path):
    module = load_script("p7_static_camera_projection.py")
    urdf = tmp_path / "camera.urdf"
    urdf.write_text(
        """<robot name="test">
  <link name="world"/><link name="camera_depth_optical_frame"/>
  <joint name="camera_fixed" type="fixed">
    <parent link="world"/><child link="camera_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>""",
        encoding="utf-8",
    )
    transform = module.camera_transform_from_urdf(urdf, {})
    projection = module.project_target(
        transform,
        module.np.array([0.0, 0.0, 1.0]),
        width=320,
        height=240,
        horizontal_fov_rad=1.047,
    )
    assert projection["inside_image"] is True
    assert projection["u_px"] == pytest.approx(160.0)
    assert projection["v_px"] == pytest.approx(120.0)


def test_p7_1_camera_centre_ray_clears_gripper_palm_with_margin():
    camera_x, camera_z = 0.03, 0.15
    pitch = math.radians(45.0)
    palm_x_min, palm_x_max = 0.085 - 0.040, 0.085 + 0.040
    palm_z_max = 0.025

    def ray_z_at_x(x):
        return camera_z - (x - camera_x) * math.tan(pitch)

    minimum_clearance = min(
        ray_z_at_x(palm_x_min) - palm_z_max,
        ray_z_at_x(palm_x_max) - palm_z_max,
    )
    assert minimum_clearance == pytest.approx(0.030)
    assert minimum_clearance >= 0.030 - 1.0e-12


def test_p7_1_capture_has_no_motion_or_planning_command_surface():
    root = Path(__file__).resolve().parents[3]
    source = (root / "test" / "p7_capture_sensor_gate.py").read_text(
        encoding="utf-8"
    )
    for prohibited in (
        "ActionClient",
        "create_publisher",
        "send_goal_async",
        "FollowJointTrajectory",
        "ApplyPlanningScene",
        "/p7/arm_motion_command",
        "/p7/gripper_command",
    ):
        assert prohibited not in source

    description = (
        root
        / "src"
        / "cs625_ap_description"
        / "urdf"
        / "cs625_active_perception.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert 'camera_mount_xyz" default="0.03 0 0.15"' in description

    launcher = (root / "test" / "run_p7_1_sensor_sim.sh").read_text(
        encoding="utf-8"
    )
    assert "launch_moveit:=false" in launcher
    assert "launch_sensor_adapter:=true" in launcher
    assert "P7_1_SIM_READY" in launcher
    assert "run_p7_adapter.sh" not in launcher
    assert "p7_1_observation_initial_positions.yaml" in launcher
    assert "CS625_P7_1_DIAGNOSTIC" in launcher
    assert "A non-acceptance world requires" in launcher

    capture_runner = (
        root / "test" / "run_p7_1_sensor_gate_capture.sh"
    ).read_text(encoding="utf-8")
    assert "CS625_P7_1_SENSOR_GATE" in capture_runner
    assert "Refusing to reuse an evidence directory" in capture_runner
    assert "--expected-windows 5" in capture_runner
    assert "p7_1_observation_initial_positions.yaml" in capture_runner
    assert "p7_1_observation_settled_positions.yaml" in capture_runner
    assert "--expected-positions-file" in capture_runner
    assert "set -eo pipefail" in capture_runner
    assert "trap finalize_evidence EXIT" in capture_runner
    assert "default=json_scalar" in source

    solver = (root / "test" / "p7_solve_static_observation_pose.py").read_text(
        encoding="utf-8"
    )
    for prohibited in ("rclpy", "moveit_msgs", "ActionClient", "FollowJointTrajectory"):
        assert prohibited not in solver

    manifest = (root / "test" / "p7_capture_manifest.py").read_text(
        encoding="utf-8"
    )
    for artifact in (
        '"test/p7_capture_sensor_gate.py"',
        '"test/p7_capture_axis_marker_diagnostic.py"',
        '"test/p7_static_camera_projection.py"',
        '"test/p7_solve_static_observation_pose.py"',
        '"test/run_p7_1_sensor_gate_capture.sh"',
        '"test/run_p7_1_sensor_sim.sh"',
    ):
        assert artifact in manifest
    assert '"src/cs625_bringup/config/p7_1_observation_settled_positions.yaml"' in manifest

    preflight = (root / "test" / "p7_capture_preflight.py").read_text(
        encoding="utf-8"
    )
    assert "--expected-positions-file" in preflight
    assert '"INITIAL_JOINT_UNSTABLE"' in preflight
    assert '"joint_spreads_rad"' in preflight

    p7_3_launcher = (
        root / "test" / "run_p7_3_e65_static_reobservation.sh"
    ).read_text(encoding="utf-8")
    assert "run_p7_1_sensor_sim.sh" in p7_3_launcher
    for prohibited in (
        "launch_moveit:=true",
        "run_p7_adapter.sh",
        "FollowJointTrajectory",
        "gripper_command",
    ):
        assert prohibited not in p7_3_launcher

    diagnostic = (
        root / "test" / "p7_capture_axis_marker_diagnostic.py"
    ).read_text(encoding="utf-8")
    assert '"evidence_class": "DIAGNOSTIC_ONLY"' in diagnostic
    assert '"acceptance_effect": "none"' in diagnostic
    for prohibited in (
        "ActionClient",
        "create_publisher",
        "send_goal_async",
        "FollowJointTrajectory",
        "ApplyPlanningScene",
    ):
        assert prohibited not in diagnostic


def test_p7_2_estimator_cannot_read_ground_truth_and_reports_unobservable_yaw():
    root = Path(__file__).resolve().parents[3]
    estimator = (root / "test" / "p7_2_estimate_cylinder_pose.py").read_text(
        encoding="utf-8"
    )
    assert "--ground-truth" not in estimator
    assert "target_ground_truth" not in estimator
    assert '"ground_truth_read": False' in estimator
    assert '"TEXTURE_YAW_UNOBSERVABLE"' in estimator
    assert '"full_se3_gate_pass": not failures' in estimator

    evaluator = (root / "test" / "p7_2_evaluate_pose_estimates.py").read_text(
        encoding="utf-8"
    )
    assert "--ground-truth" in evaluator
    assert "evaluation_only_after_estimates_were_immutable" in evaluator
    assert '"add_s_m"' in evaluator
