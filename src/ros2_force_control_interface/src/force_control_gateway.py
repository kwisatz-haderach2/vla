#!/usr/bin/env python3
"""ROS 2 force-control gateway for the VLA writing demo.

This node deliberately stops at a clean interface boundary: it normalises
commands, estimates contact, and publishes the measured/target state.  The
actual UR5e force loop can subscribe to ``target_wrench_topic`` and publish a
``geometry_msgs/WrenchStamped`` on ``measured_wrench_topic``.  No hardware API
is assumed, so the same gateway runs in Gazebo, with a real force/torque sensor,
or beside a teammate's controller.

The standard-message mirror topics are the ROS 1 <-> ROS 2 bridge ABI.  They
avoid requiring a custom ROS 1 package merely to exchange the live pen state.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Wrench, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float64, String

from ros2_force_control_interface.msg import ContactState, ForceCommand, ForceState
from ros2_force_control_interface.srv import SetCompliance, SetForceControl


class ForceControlGateway(Node):
    """A hardware-agnostic command/state gateway and contact estimator."""

    MODE_DISABLED = 0
    MODE_FORCE = 1
    MODE_COMPLIANCE = 2
    MODE_MONITOR = 3

    def __init__(self) -> None:
        super().__init__("force_control_gateway")

        # ``use_sim_time`` is normally declared by rclpy's clock, but declaring
        # it conditionally also makes direct invocation predictable on Humble.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self.command_topic = self._declare("command_topic", "/vla/force_command")
        self.measured_wrench_topic = self._declare(
            "measured_wrench_topic", "/vla/measured_wrench"
        )
        self.force_state_topic = self._declare("force_state_topic", "/vla/force_state")
        self.contact_state_topic = self._declare(
            "contact_state_topic", "/vla/contact_state"
        )
        self.target_wrench_topic = self._declare(
            "target_wrench_topic", "/vla/force_target_wrench"
        )
        self.set_force_service = self._declare(
            "set_force_service", "/vla/set_force_control"
        )
        self.set_compliance_service = self._declare(
            "set_compliance_service", "/vla/set_compliance"
        )
        self.pen_pose_topic = self._declare("pen_pose_topic", "/vla/pen_pose")
        self.contact_threshold = max(
            0.0, float(self._declare("contact_threshold", 1.0))
        )
        self.force_axis = str(self._declare("force_axis", "z")).lower()
        if self.force_axis not in ("x", "y", "z"):
            self.get_logger().warning("force_axis must be x, y, or z; using z")
            self.force_axis = "z"
        self.publish_rate = max(1.0, float(self._declare("publish_rate", 50.0)))
        self.frame_id = str(self._declare("frame_id", "paper_frame"))

        # Standard-message mirrors.  These names are intentionally boring: a
        # ros1_bridge dynamic bridge can carry them without custom converters.
        self.bridge_command_wrench_topic = self._declare(
            "bridge_command_wrench_topic", "/vla/force_cmd_wrench"
        )
        self.bridge_target_force_topic = self._declare(
            "bridge_target_force_topic", "/vla/force_target"
        )
        self.bridge_enable_topic = self._declare(
            "bridge_enable_topic", "/vla/force_enable"
        )
        self.bridge_mode_topic = self._declare("bridge_mode_topic", "/vla/force_mode")
        self.bridge_measured_wrench_topic = self._declare(
            "bridge_measured_wrench_topic", "/vla/force_state_wrench"
        )
        self.bridge_contact_topic = self._declare(
            "bridge_contact_topic", "/vla/pen_contact"
        )
        self.bridge_status_topic = self._declare("bridge_status_topic", "/vla/status")

        qos = QoSProfile(depth=10)

        # Internal/custom interface publishers.
        self.force_state_pub = self.create_publisher(ForceState, self.force_state_topic, qos)
        self.contact_state_pub = self.create_publisher(
            ContactState, self.contact_state_topic, qos
        )
        self.target_wrench_pub = self.create_publisher(
            WrenchStamped, self.target_wrench_topic, qos
        )

        # Bridge ABI publishers.
        self.bridge_state_wrench_pub = self.create_publisher(
            WrenchStamped, self.bridge_measured_wrench_topic, qos
        )
        self.bridge_contact_pub = self.create_publisher(Bool, self.bridge_contact_topic, qos)
        self.bridge_status_pub = self.create_publisher(String, self.bridge_status_topic, qos)

        # Custom command and standard command mirror subscribers.
        self.create_subscription(ForceCommand, self.command_topic, self._on_command, qos)
        self.create_subscription(
            WrenchStamped,
            self.bridge_command_wrench_topic,
            self._on_bridge_wrench_command,
            qos,
        )
        self.create_subscription(
            Float64, self.bridge_target_force_topic, self._on_bridge_target, qos
        )
        self.create_subscription(Bool, self.bridge_enable_topic, self._on_bridge_enable, qos)
        self.create_subscription(String, self.bridge_mode_topic, self._on_bridge_mode, qos)

        # Measured wrench is normally supplied by a sensor/controller node.
        self.create_subscription(
            WrenchStamped,
            self.measured_wrench_topic,
            self._on_measured_wrench,
            qos,
        )
        # The ROS 1 planner publishes this pose while writing.  Keeping the
        # subscription here gives a force controller a stable synchronization
        # point without coupling this package to MoveIt or Gazebo.
        self.create_subscription(PoseStamped, self.pen_pose_topic, self._on_pen_pose, qos)

        self.create_service(SetForceControl, self.set_force_service, self._on_set_force)
        self.create_service(SetCompliance, self.set_compliance_service, self._on_set_compliance)

        self._measured_wrench = Wrench()
        self._last_measured_header = None
        self._measured_frame_id = ""
        self._last_pen_pose: Optional[PoseStamped] = None
        self._target_normal_force = 0.0
        self._force_tolerance = 0.5
        self._max_normal_velocity = 0.01
        self._enabled = False
        self._controller_mode = "disabled"
        self._source = "startup"
        self._compliance_active = False
        self._normal_stiffness = 0.0
        self._tangential_stiffness = 0.0
        self._damping = 0.0
        self._max_displacement = 0.0
        self._timer = self.create_timer(1.0 / self.publish_rate, self._publish_state)

        self.get_logger().info(
            "force gateway ready (command=%s, measured=%s, bridge pose=%s)"
            % (self.command_topic, self.measured_wrench_topic, self.pen_pose_topic)
        )

    def _declare(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _copy_wrench(source: Wrench) -> Wrench:
        result = Wrench()
        result.force.x = float(source.force.x)
        result.force.y = float(source.force.y)
        result.force.z = float(source.force.z)
        result.torque.x = float(source.torque.x)
        result.torque.y = float(source.torque.y)
        result.torque.z = float(source.torque.z)
        return result

    def _normal_from_wrench(self, wrench: Wrench) -> float:
        value = getattr(wrench.force, self.force_axis, 0.0)
        # A force sensor may report either sign depending on how it is mounted;
        # this interface exposes a positive normal-force magnitude.
        return abs(float(value)) if math.isfinite(float(value)) else 0.0

    def _mode_number(self) -> int:
        if not self._enabled:
            return self.MODE_DISABLED
        mode = self._controller_mode.strip().lower()
        if mode in ("compliance", "impedance", "admittance"):
            return self.MODE_COMPLIANCE
        if mode in ("monitor", "observe", "measurement"):
            return self.MODE_MONITOR
        return self.MODE_FORCE

    def _resolve_target(self, command: ForceCommand) -> float:
        explicit = float(command.target_normal_force)
        if explicit > 0.0:
            return explicit
        return self._normal_from_wrench(command.desired_wrench)

    def _on_command(self, command: ForceCommand) -> None:
        self._target_normal_force = max(0.0, self._resolve_target(command))
        self._force_tolerance = max(0.0, float(command.force_tolerance))
        if command.max_normal_velocity > 0.0:
            self._max_normal_velocity = float(command.max_normal_velocity)
        self._enabled = bool(command.enable)
        self._controller_mode = command.controller_mode or (
            "force" if self._enabled else "disabled"
        )
        self._source = command.source or "custom_command"
        if command.reset_integrator:
            # The low-level controller owns the integrator.  This edge is still
            # surfaced as a log so the teammate can reset it in their node.
            self.get_logger().debug(
                "force integrator reset requested by %s" % self._source
            )

    def _on_bridge_wrench_command(self, message: WrenchStamped) -> None:
        self._target_normal_force = self._normal_from_wrench(message.wrench)
        self._source = "ros1_bridge_wrench"

    def _on_bridge_target(self, message: Float64) -> None:
        self._target_normal_force = max(0.0, float(message.data))
        self._source = "ros1_bridge_target"

    def _on_bridge_enable(self, message: Bool) -> None:
        self._enabled = bool(message.data)
        if not self._enabled:
            self._controller_mode = "disabled"
        elif self._controller_mode == "disabled":
            self._controller_mode = "force"
        self._source = "ros1_bridge_enable"

    def _on_bridge_mode(self, message: String) -> None:
        self._controller_mode = message.data or ("force" if self._enabled else "disabled")
        self._source = "ros1_bridge_mode"

    def _on_measured_wrench(self, message: WrenchStamped) -> None:
        self._measured_wrench = self._copy_wrench(message.wrench)
        self._last_measured_header = message.header
        self._measured_frame_id = message.header.frame_id

    def _on_pen_pose(self, message: PoseStamped) -> None:
        self._last_pen_pose = message

    def _on_set_force(self, request, response):
        self._enabled = bool(request.enable)
        self._target_normal_force = max(0.0, float(request.target_normal_force))
        self._force_tolerance = max(0.0, float(request.force_tolerance))
        if request.max_normal_velocity > 0.0:
            self._max_normal_velocity = float(request.max_normal_velocity)
        self._controller_mode = request.controller_mode or (
            "force" if self._enabled else "disabled"
        )
        self._source = "set_force_control"
        if request.reset_integrator:
            self.get_logger().debug("force integrator reset requested by service")
        response.accepted = True
        response.message = "force command accepted"
        response.active_target_normal_force = self._target_normal_force
        response.active_mode = self._mode_number()
        return response

    def _on_set_compliance(self, request, response):
        self._compliance_active = bool(request.enable)
        self._normal_stiffness = max(0.0, float(request.normal_stiffness))
        self._tangential_stiffness = max(0.0, float(request.tangential_stiffness))
        self._damping = max(0.0, float(request.damping))
        self._max_displacement = max(0.0, float(request.max_displacement))
        if self._compliance_active:
            self._enabled = True
            self._controller_mode = "compliance"
        elif self._controller_mode == "compliance":
            self._controller_mode = "force" if self._enabled else "disabled"
        self._source = "set_compliance"
        response.accepted = True
        response.active = self._compliance_active
        response.message = "compliance settings accepted"
        return response

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        measured = self._normal_from_wrench(self._measured_wrench)
        error = self._target_normal_force - measured
        in_contact = measured >= self.contact_threshold
        mode = self._mode_number()

        target = Wrench()
        target.force.z = self._target_normal_force

        state = ForceState()
        state.header.stamp = now
        state.header.frame_id = self.frame_id
        state.measured_wrench = self._copy_wrench(self._measured_wrench)
        state.target_wrench = self._copy_wrench(target)
        state.normal_force = measured
        state.target_normal_force = self._target_normal_force
        state.normal_force_error = error
        state.force_tolerance = self._force_tolerance
        state.contact_threshold = self.contact_threshold
        state.contact_depth = 0.0  # displacement belongs to the low-level controller
        state.in_contact = in_contact
        state.controller_enabled = self._enabled
        state.mode = mode
        state.controller_state = self._status_text(in_contact)
        state.source = self._source
        self.force_state_pub.publish(state)

        contact = ContactState()
        contact.header = state.header
        contact.in_contact = in_contact
        contact.normal_force = measured
        contact.threshold = self.contact_threshold
        contact.force_error = error
        contact.mode = mode
        contact.source = self._source
        self.contact_state_pub.publish(contact)

        target_stamped = WrenchStamped()
        target_stamped.header = state.header
        target_stamped.wrench = self._copy_wrench(target)
        self.target_wrench_pub.publish(target_stamped)

        # Bridge mirrors consumed by ROS 1 nodes.
        measured_stamped = WrenchStamped()
        measured_stamped.header = state.header
        if self._measured_frame_id:
            measured_stamped.header.frame_id = self._measured_frame_id
        measured_stamped.wrench = self._copy_wrench(self._measured_wrench)
        self.bridge_state_wrench_pub.publish(measured_stamped)

        contact_bool = Bool()
        contact_bool.data = in_contact
        self.bridge_contact_pub.publish(contact_bool)

        status = String()
        status.data = self._status_text(in_contact)
        self.bridge_status_pub.publish(status)

    def _status_text(self, in_contact: bool) -> str:
        if not self._enabled:
            return "DISABLED"
        if self._compliance_active:
            return "COMPLIANCE_CONTACT" if in_contact else "COMPLIANCE_SEARCH"
        if self._mode_number() == self.MODE_MONITOR:
            return "MONITOR_CONTACT" if in_contact else "MONITORING"
        return "FORCE_CONTACT" if in_contact else "FORCE_SEARCH"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceControlGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
