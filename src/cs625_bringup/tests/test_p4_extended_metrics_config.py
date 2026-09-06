from pathlib import Path


def test_p4_loop_subscribes_to_the_p3_motion_status_contract():
    config = (Path(__file__).resolve().parents[1] / "config" / "p4_sim_loop.yaml").read_text(
        encoding="utf-8"
    )
    assert "motion_status_topic: /motion/status" in config
