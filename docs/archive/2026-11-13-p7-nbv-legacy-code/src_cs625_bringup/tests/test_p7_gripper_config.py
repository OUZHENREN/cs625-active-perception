from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_parallel_gripper_is_application_owned_and_gz_controlled():
    text = (
        ROOT / "cs625_ap_description" / "urdf" / "cs625_parallel_gripper.xacro"
    ).read_text(encoding="utf-8")
    assert "gz_ros2_control/GazeboSimSystem" in text
    assert "gripper_right_finger_joint" in text
    assert "<mimic joint=" in text
    assert "<command_interface name=\"position\"/>" in text


def test_attachment_assisted_mechanism_is_explicitly_gazebo_scoped():
    wrapper = (
        ROOT / "cs625_ap_description" / "urdf" / "cs625_active_perception.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert "gz-sim-detachable-joint-system" in wrapper
    assert "wrist_3_link" in wrapper
    assert "target_object" in wrapper
    assert "/p7/attachment/detach" in wrapper
    assert "/p7/attachment/attach" in wrapper


def test_sim_control_spawns_the_matching_gripper_controller():
    config = (ROOT / "cs625_bringup" / "config" / "sim_controllers.yaml").read_text(encoding="utf-8")
    launch = (ROOT / "cs625_bringup" / "launch" / "sim_control.launch.py").read_text(encoding="utf-8")
    assert "forward_command_controller/ForwardCommandController" in config
    assert "gripper_right_finger_joint" in config
    assert '"gripper_controller"' in launch
