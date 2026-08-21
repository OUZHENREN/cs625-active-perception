"""Explicitly authorized real-profile executor built on the shared MoveIt path."""

from __future__ import annotations

from cs625_motion_adapter.sim_view_executor import SimViewExecutor


class RealViewExecutor(SimViewExecutor):
    """Use a freshly collision-checked MoveIt trajectory only after all R4 gates.

    Construction or subscription never commands the robot.  The inherited action
    client is used only after the profile, execute, confirmation and operator
    authorization parameters all validate; otherwise every selection is logged
    as a refusal.
    """

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("r4_authorized", False)
        self.declare_parameter("operator_id", "")
        self.declare_parameter("approval_id", "")
        execute = bool(self.get_parameter("execute").value)
        confirmation = bool(self.get_parameter("require_confirmation").value)
        authorized = bool(self.get_parameter("r4_authorized").value)
        operator = str(self.get_parameter("operator_id").value).strip()
        approval = str(self.get_parameter("approval_id").value).strip()
        velocity = float(self.get_parameter("max_velocity_scale").value)
        acceleration = float(self.get_parameter("max_acceleration_scale").value)
        self._enabled = (
            self._profile == "real"
            and execute
            and not confirmation
            and authorized
            and bool(operator)
            and bool(approval)
            and 0.0 < velocity <= 0.10
            and 0.0 < acceleration <= 0.10
        )

    def _on_selection(self, selection) -> None:
        if self._phase != "idle":
            return
        if not self._enabled:
            self._selection = selection
            self._finish(False, "R4_AUTHORIZATION_REQUIRED")
            return
        super()._on_selection(selection)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = RealViewExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
