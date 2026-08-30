"""MoveIt 1 execution adapter for the ROS1 writing pipeline."""

from __future__ import annotations

import inspect
import logging
import math
import sys
import time
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .data_types import Stroke
from .joint_state_utils import (
    UR5E_JOINT_LIMITS,
    canonical_angle,
    invalid_joint_state,
    joint_limits,
    normalize_trajectory,
)
from .trajectory_builder import PaperWaypoint, TrajectoryBuilder

LOG = logging.getLogger(__name__)


class MoveItExecutor:
    """Own the MoveGroup and execute a complete sentence Cartesian path.

    The class imports ROS/MoveIt lazily so all parser/layout modules remain
    usable in a plain Python test.  Set ``~dry_run:=true`` for a trajectory
    preview without a running move_group; the normal launch uses real MoveIt.
    """

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        status_cb: Optional[Callable[[str], None]] = None,
        dry_run: bool = False,
        tf_buffer: Any = None,
    ):
        self.config = dict(config or {})
        self.status_cb = status_cb
        self.dry_run = bool(dry_run)
        self.tf_buffer = tf_buffer
        self.robot = None
        self.scene = None
        self.group = None
        self.active_eef_link = str(self._get("moveit.eef_link", "pen_tip"))
        self.planning_frame = str(self._get("moveit.base_frame", "base_link"))
        configured_joint_names = self._get("moveit.joint_names", None)
        if isinstance(configured_joint_names, Sequence) and not isinstance(configured_joint_names, (str, bytes)):
            self.joint_names = tuple(str(name) for name in configured_joint_names)
        else:
            self.joint_names = tuple(UR5E_JOINT_LIMITS.keys())
        self.joint_limits = self._get("moveit.joint_limits", None)
        if not isinstance(self.joint_limits, Mapping):
            self.joint_limits = self._get("motion.joint_limits", None)
        self.joint_state_tolerance = float(self._get("motion.joint_state_tolerance", 0.10))
        self.joint_limit_margin = float(self._get("motion.joint_limit_margin", 1e-6))
        self._last_recovery_time = 0.0
        self._last_tf_error = ""
        self.initialized = False
        self._roscpp_initialized = False

    def _get(self, dotted: str, default: Any = None) -> Any:
        value: Any = self.config
        for key in dotted.split("."):
            if not isinstance(value, Mapping) or key not in value:
                return default
            value = value[key]
        return value

    def _status(self, text: str) -> None:
        LOG.info("%s", text)
        if self.status_cb:
            try:
                self.status_cb(text)
            except Exception:
                LOG.exception("status callback failed")

    def initialize(self) -> bool:
        if self.initialized or self.dry_run:
            self.initialized = True
            return True
        try:
            import moveit_commander  # type: ignore
            import rospy  # type: ignore
        except ImportError as exc:
            self._status("MoveIt import failed: {}".format(exc))
            return False
        try:
            if not self._roscpp_initialized:
                moveit_commander.roscpp_initialize(sys.argv)
                self._roscpp_initialized = True
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            group_name = str(self._get("moveit.planning_group", "manipulator"))
            self.group = moveit_commander.MoveGroupCommander(group_name)
            self.group.set_planning_time(float(self._get("motion.planning_time", 8.0)))
            self.group.set_max_velocity_scaling_factor(float(self._get("motion.velocity_scaling", 0.10)))
            self.group.set_max_acceleration_scaling_factor(float(self._get("motion.acceleration_scaling", 0.10)))
            planner_id = self._get("moveit.planner_id", "")
            if planner_id:
                try:
                    self.group.set_planner_id(str(planner_id))
                except Exception:
                    LOG.debug("planner id %s is not available", planner_id, exc_info=True)
            try:
                self.planning_frame = self.group.get_planning_frame()
            except Exception:
                pass
            requested = str(self._get("moveit.eef_link", "pen_tip"))
            fallback = str(self._get("moveit.fallback_eef_link", "tool0"))
            try:
                self.group.set_end_effector_link(requested)
                self.active_eef_link = requested
            except Exception:
                LOG.warning("MoveIt group does not expose %s; using %s", requested, fallback)
                try:
                    self.group.set_end_effector_link(fallback)
                except Exception:
                    pass
                self.active_eef_link = fallback
            self._add_table_collision()
            self.initialized = True
            self._status("MoveIt ready (group={}, eef={})".format(group_name, self.active_eef_link))
            return True
        except Exception as exc:
            self._status("MoveIt initialization failed: {}".format(exc))
            LOG.exception("MoveIt initialization failed")
            return False

    def _add_table_collision(self) -> None:
        if self.scene is None:
            return
        try:
            from geometry_msgs.msg import PoseStamped  # type: ignore
            pose = PoseStamped()
            pose.header.frame_id = self.planning_frame
            xyz = list(self._get("moveit.table_pose", [0.0, 0.0, 0.28]))
            pose.pose.orientation.w = 1.0
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [float(v) for v in xyz[:3]]
            dims = list(self._get("moveit.table_dimensions", [0.9, 0.7, 0.56]))
            name = str(self._get("moveit.table_collision_name", "writing_table"))
            self.scene.add_box(name, pose, size=tuple(float(v) for v in dims[:3]))
        except Exception:
            LOG.warning("Could not add table to PlanningScene", exc_info=True)

    # ------------------------------------------------------------------
    # Joint-state canonicalisation and simulator recovery
    # ------------------------------------------------------------------
    def _active_joint_names(self) -> Tuple[str, ...]:
        """Return the MoveIt group's actuated joint order.

        ``get_active_joints`` is available in MoveIt 1 and excludes fixed
        joints.  A few vendor builds expose only ``get_joints``; the configured
        UR5e order is the final fallback and is also useful in dry-run tests.
        """

        if self.group is not None:
            for method_name in ("get_active_joints", "get_joints"):
                try:
                    method = getattr(self.group, method_name)
                    values = method()
                    if values:
                        names = tuple(str(name) for name in values)
                        # MoveIt may include a virtual/fixed joint in
                        # get_joints(); retain only names for which we know a
                        # finite limit, unless all names are unknown.
                        known = tuple(name for name in names if name in UR5E_JOINT_LIMITS)
                        if known:
                            return known
                        return names
                except Exception:
                    LOG.debug("Could not query group joint names", exc_info=True)
        return self.joint_names

    def _current_joint_state(self) -> Tuple[Tuple[str, ...], List[float]]:
        """Read the current MoveIt state with a stable name/value pairing."""

        names = self._active_joint_names()
        if self.group is None:
            return names, []
        try:
            values = [float(value) for value in self.group.get_current_joint_values()]
        except Exception:
            return names, []
        # Some MoveIt wrappers return a value vector for all model joints even
        # when ``get_active_joints`` returned only the six arm joints.  Pair the
        # vectors conservatively instead of raising and losing the recovery
        # opportunity.
        if len(values) != len(names):
            if len(values) >= len(self.joint_names):
                names = self.joint_names
                values = values[:len(names)]
            else:
                names = names[:len(values)]
        return names, values

    def _state_out_of_bounds(self) -> Dict[str, float]:
        names, values = self._current_joint_state()
        if not values or len(names) != len(values):
            return {}
        return invalid_joint_state(
            names,
            values,
            limits=self.joint_limits,
            tolerance=self.joint_state_tolerance,
        )

    def _canonical_current_state(self) -> Dict[str, float]:
        names, values = self._current_joint_state()
        if not values or len(names) != len(values):
            return {}
        # Preserve every already-bounded value exactly.  Re-canonicalising a
        # valid state such as +5.5 rad to its mathematically equivalent -0.78
        # rad would make the recovery service command an unnecessary physical
        # rotation.  Only materially invalid/multi-turn samples are folded.
        result: Dict[str, float] = {}
        for name, value in zip(names, values):
            bound = joint_limits(name, self.joint_limits)
            if bound is None:
                if not math.isfinite(value):
                    return {}
                result[name] = float(value)
                continue
            if math.isfinite(value) and bound[0] <= value <= bound[1]:
                result[name] = float(value)
            else:
                try:
                    result[name] = canonical_angle(
                        value,
                        bound[0],
                        bound[1],
                        reference=None,
                        margin=self.joint_limit_margin,
                    )
                except (TypeError, ValueError):
                    return {}
        return result

    def _controller_names_for_recovery(self) -> List[str]:
        configured = self._get("recovery.trajectory_controllers", None)
        if isinstance(configured, Sequence) and not isinstance(configured, (str, bytes)):
            return [str(name) for name in configured if str(name)]
        configured_name = self._get("moveit.controller_name", None)
        if configured_name:
            return [str(configured_name)]
        # This is the controller launched by writing_scene.launch.  Include the
        # effort name as a fallback so the same recovery path remains usable
        # when the ROS2/force branch hands control back to ROS1.
        return ["pos_joint_traj_controller", "eff_joint_traj_controller", "scaled_pos_joint_traj_controller"]

    def _running_controller_names(self, candidates: Sequence[str]) -> List[str]:
        """Query controller_manager and keep only running arm controllers.

        Querying avoids a strict switch failure when the launch contains only
        ``pos_joint_traj_controller`` but the force branch's effort controller
        is also listed as a possible hand-off target.  If the list service is
        unavailable, return the candidates and let the switch service apply
        its configured strictness.
        """

        names = [str(name) for name in candidates if str(name)]
        if not names:
            return []
        service_name = str(self._get("recovery.list_controllers_service", "/controller_manager/list_controllers"))
        try:
            import rospy  # type: ignore
            from controller_manager_msgs.srv import ListControllers  # type: ignore
            timeout = float(self._get("recovery.service_timeout", 2.0))
            rospy.wait_for_service(service_name, timeout=timeout)
            response = rospy.ServiceProxy(service_name, ListControllers)()
            states = getattr(response, "controller", None)
            if states is None:
                states = getattr(response, "controllers", None)
            if states is None:
                return names
            running = []
            candidate_set = set(names)
            for state in states:
                state_name = str(getattr(state, "name", ""))
                state_value = str(getattr(state, "state", "")).lower()
                if state_name in candidate_set and state_value in ("running", "active"):
                    running.append(state_name)
            return running
        except Exception:
            LOG.debug("Could not query running controllers", exc_info=True)
            return names

    @staticmethod
    def _service_response_ok(response: Any) -> bool:
        """Interpret ROS service responses across generated-message versions."""

        if response is None:
            return True
        # ``SwitchController`` uses ``ok`` while Gazebo's
        # ``SetModelConfiguration`` uses ``success``.  Check both instead of
        # treating a response with an absent ``ok`` field as unconditional
        # success (which used to hide failed Gazebo resets).
        seen = False
        for field in ("ok", "success"):
            value = getattr(response, field, None)
            if value is not None:
                seen = True
                if not bool(value):
                    return False
        return True if seen else True

    def _switch_controllers(self, stop: Sequence[str], start: Sequence[str]) -> Tuple[bool, List[str]]:
        """Stop/start trajectory controllers around a Gazebo state reset."""

        if not stop and not start:
            return True, []
        service_name = str(self._get("recovery.switch_controller_service", "/controller_manager/switch_controller"))
        try:
            import rospy  # type: ignore
            from controller_manager_msgs.srv import SwitchController  # type: ignore
            timeout = float(self._get("recovery.service_timeout", 2.0))
            rospy.wait_for_service(service_name, timeout=timeout)
            proxy = rospy.ServiceProxy(service_name, SwitchController)
            # Keyword invocation works with the generated request class and
            # with rospy's convenience proxy.  Fall back to positional fields
            # for older controller_manager releases.
            try:
                response = proxy(
                    start_controllers=list(start),
                    stop_controllers=list(stop),
                    strictness=int(self._get("recovery.switch_strictness", 1)),
                    start_asap=True,
                    # controller_manager_msgs/SwitchController.timeout is a
                    # float64 (seconds), not a ROS Duration.
                    timeout=float(timeout),
                )
            except TypeError:
                response = proxy(
                    list(start),
                    list(stop),
                    int(self._get("recovery.switch_strictness", 1)),
                    True,
                    float(timeout),
                )
            return self._service_response_ok(response), list(stop)
        except Exception as exc:
            LOG.warning("Controller switch service %s unavailable: %s", service_name, exc)
            return False, []

    def _reset_gazebo_joint_state(self, canonical: Mapping[str, float]) -> bool:
        """Set the simulator's six arm joints to bounded equivalent angles."""

        service_name = str(self._get("recovery.set_model_configuration_service", "/gazebo/set_model_configuration"))
        model_name = str(self._get("scene.robot_model", "ur5e_writing"))
        urdf_param = str(self._get("recovery.urdf_param_name", "robot_description"))
        active_names = self._active_joint_names()
        names = [name for name in active_names if name in canonical]
        if not names:
            names = [name for name in self.joint_names if name in canonical]
        if not names:
            names = [str(name) for name in canonical.keys()]
        values = [float(canonical[name]) for name in names]
        if len(names) < 1:
            return False
        try:
            import rospy  # type: ignore
            from gazebo_msgs.srv import SetModelConfiguration  # type: ignore
            timeout = float(self._get("recovery.service_timeout", 2.0))
            rospy.wait_for_service(service_name, timeout=timeout)
            proxy = rospy.ServiceProxy(service_name, SetModelConfiguration)
            try:
                response = proxy(
                    model_name=model_name,
                    urdf_param_name=urdf_param,
                    joint_names=names,
                    joint_positions=values,
                )
            except TypeError:
                response = proxy(model_name, urdf_param, names, values)
            return self._service_response_ok(response)
        except Exception as exc:
            LOG.warning("Gazebo joint-state recovery via %s failed: %s", service_name, exc)
            return False

    def _recover_invalid_joint_state(self) -> bool:
        """Repair a multi-turn Gazebo state before asking MoveIt to plan.

        The operation is deliberately conservative: valid states are left
        untouched, while an invalid state is reset only to an equivalent angle
        inside the configured UR5e limits.  A short cooldown prevents repeated
        service calls while Gazebo publishes a few stale joint-state samples.
        """

        if self.dry_run or self.group is None:
            return True
        invalid = self._state_out_of_bounds()
        if not invalid:
            return True
        now = time.monotonic()
        cooldown = float(self._get("recovery.cooldown", 0.25))
        if now - self._last_recovery_time < max(0.0, cooldown):
            return False
        self._last_recovery_time = now
        canonical = self._canonical_current_state()
        if not canonical:
            LOG.warning("Invalid joint state %s but no readable state vector", invalid)
            return False
        self._status("recovering bounded joint state: {}".format(
            ", ".join("{}={:.3f}".format(name, value) for name, value in invalid.items())))
        try:
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception:
            pass

        controller_names = self._running_controller_names(self._controller_names_for_recovery())
        stopped_ok, stopped = self._switch_controllers(controller_names, [])
        # A missing switch service should not prevent trying the Gazebo reset:
        # some simulated setups do not expose controller_manager, and the
        # state service itself is still sufficient there.
        reset_ok = self._reset_gazebo_joint_state(canonical)
        if stopped:
            restart_ok, _ = self._switch_controllers([], stopped)
        else:
            restart_ok = True
        if not reset_ok:
            self._status("joint-state recovery failed")
            return False

        # Wait until the planning scene consumes a bounded joint_states sample.
        # ``rospy.sleep`` keeps simulated time semantics; wall-clock fallback
        # prevents an infinite wait when /clock is paused or absent.
        timeout = max(0.2, float(self._get("recovery.state_timeout", 3.0)))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current_invalid = self._state_out_of_bounds()
            if not current_invalid:
                if not stopped_ok:
                    LOG.info("Recovered joint state without controller switch")
                if not restart_ok:
                    LOG.warning("Joint state recovered but controller restart reported failure")
                self._status("joint state canonicalized")
                return True
            # Use wall time rather than rospy.sleep: when simulation time is
            # paused, rospy.sleep can wait forever and defeat the wall-clock
            # recovery deadline above.
            time.sleep(0.05)
        LOG.warning("Joint state remained outside bounds after recovery: %s", self._state_out_of_bounds())
        self._status("joint-state recovery timed out")
        return False

    def _normalize_trajectory(self, trajectory: Any) -> Any:
        """Normalize a MoveIt result against the current bounded state."""

        if trajectory is None or self.dry_run:
            return trajectory
        reference = self._canonical_current_state()
        try:
            normalized = normalize_trajectory(
                trajectory,
                limits=self.joint_limits,
                reference=reference,
                margin=self.joint_limit_margin,
            )
            if normalized is not trajectory:
                LOG.debug("Normalized trajectory joint angles before execution")
            return normalized
        except Exception:
            LOG.warning("Could not normalize MoveIt trajectory; using original", exc_info=True)
            return trajectory

    def move_to_ready_pose(self) -> bool:
        """Move to the safe folded/up pose before writing."""

        if not self.initialize():
            return False
        if self.dry_run:
            self._status("dry-run: ready pose skipped")
            return True
        if self.group is None:
            return False
        if not self._recover_invalid_joint_state():
            LOG.error("Cannot plan ready pose from an invalid joint state")
            return False
        try:
            # The official UR5e SRDF supplies the ``up`` state.  A direct joint
            # target fallback keeps this wrapper usable with a custom SRDF.
            try:
                self.group.set_named_target("up")
            except Exception:
                self.group.set_joint_value_target({
                    "shoulder_pan_joint": 0.0,
                    "shoulder_lift_joint": -1.5707,
                    "elbow_joint": 0.0,
                    "wrist_1_joint": -1.5707,
                    "wrist_2_joint": 0.0,
                    "wrist_3_joint": 0.0,
                })
            ok = bool(self.group.go(wait=True))
            self.group.stop()
            self.group.clear_pose_targets()
            if ok:
                self._status("ready pose reached")
                return True

            # The Gazebo position controller can report a transient
            # PATH_TOLERANCE_VIOLATED even though it continues following the
            # command and reaches the requested joint target (this is common
            # immediately after startup when its first state sample is stale).
            # Verify the physical state before declaring the whole writing
            # request failed.  This is deliberately limited to the ready
            # move; Cartesian execution still has to report a valid result.
            settle = max(0.0, float(self._get("motion.ready_settle_time", 0.8)))
            deadline = time.monotonic() + settle
            tolerance = max(1e-3, float(self._get("motion.ready_joint_tolerance", 0.12)))
            target_values: Dict[str, float] = {}
            try:
                getter = getattr(self.group, "get_named_target_values", None)
                if getter is not None:
                    target_values = {str(k): float(v) for k, v in (getter("up") or {}).items()}
            except Exception:
                target_values = {}
            if not target_values:
                # ``get_named_target_values`` is missing in a few MoveIt 1
                # Python builds.  Keep the local SRDF's explicit ``up`` state
                # as a deterministic fallback rather than losing the final
                # verification path.
                target_values = {
                    "shoulder_pan_joint": 0.0,
                    "shoulder_lift_joint": -1.5707,
                    "elbow_joint": 0.0,
                    "wrist_1_joint": -1.5707,
                    "wrist_2_joint": 0.0,
                    "wrist_3_joint": 0.0,
                }
            while time.monotonic() < deadline:
                names, values = self._current_joint_state()
                if target_values and len(names) == len(values):
                    current = {name: value for name, value in zip(names, values)}
                    reached = True
                    for name, target in target_values.items():
                        if name not in current:
                            continue
                        # Joint values are finite and the writing limits are
                        # narrow, so a direct error is sufficient here.  The
                        # wrapped equivalent keeps this check valid for a
                        # vendor controller that publishes one extra turn.
                        error = abs(float(current[name]) - float(target))
                        error = min(error, abs(error - 2.0 * math.pi), abs(error + 2.0 * math.pi))
                        if error > tolerance:
                            reached = False
                            break
                    if reached:
                        LOG.warning("ready controller returned failure but target joints are reached; continuing")
                        self._status("ready pose reached (controller warning)")
                        return True
                try:
                    import rospy  # type: ignore
                    rospy.sleep(0.05)
                except Exception:
                    time.sleep(0.05)
            self._status("ready pose failed")
            return False
        except Exception as exc:
            self._status("ready pose error: {}".format(exc))
            LOG.exception("ready pose failed")
            return False

    def current_orientation(self) -> Tuple[float, float, float, float]:
        if self.group is None:
            return tuple(float(v) for v in self._get("motion.orientation", [0, 0, 0, 1]))  # type: ignore[return-value]
        try:
            q = self.group.get_current_pose().pose.orientation
            return (float(q.x), float(q.y), float(q.z), float(q.w))
        except Exception:
            value = list(self._get("motion.orientation", [0, 0, 0, 1]))
            return tuple(float(v) for v in value[:4])  # type: ignore[return-value]

    def _paper_pose(self, waypoint: PaperWaypoint) -> Any:
        from geometry_msgs.msg import PoseStamped  # type: ignore
        pose = PoseStamped()
        pose.header.frame_id = str(self._get("paper.frame", "paper_frame"))
        pose.pose.position.x = float(waypoint.u)
        pose.pose.position.y = float(-waypoint.v)
        pose.pose.position.z = float(waypoint.z)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = waypoint.orientation
        return pose

    def _transform_waypoints(self, waypoints: Sequence[PaperWaypoint]) -> List[Any]:
        """Apply paper_frame→planning_frame TF to every waypoint."""

        if not waypoints:
            return []
        self._last_tf_error = ""
        try:
            import rospy  # type: ignore
            import tf2_geometry_msgs  # type: ignore
            from geometry_msgs.msg import Pose  # type: ignore
            paper_frame = str(self._get("paper.frame", "paper_frame"))
            if self.tf_buffer is None:
                import tf2_ros  # type: ignore
                self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
                self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, paper_frame, rospy.Time(0), rospy.Duration(2.0))
            output: List[Any] = []
            for waypoint in waypoints:
                transformed = tf2_geometry_msgs.do_transform_pose(self._paper_pose(waypoint), transform)
                output.append(transformed.pose)
            return output
        except Exception as exc:
            self._last_tf_error = str(exc)
            # A raw paper-frame pose is not a safe substitute in a live robot:
            # it would silently reinterpret sheet coordinates as base_link
            # coordinates.  Keep the old fallback for dry-run/offline previews
            # and for an explicit opt-in, but make the real path fail closed.
            allow_fallback = self.dry_run or bool(self._get("moveit.allow_untransformed_waypoints", False))
            if allow_fallback:
                LOG.warning("TF paper_frame→%s unavailable (%s); using paper coordinates for preview", self.planning_frame, exc)
                return [TrajectoryBuilder.to_pose(wp) for wp in waypoints]
            LOG.error("TF paper_frame→%s unavailable (%s); refusing untransformed motion", self.planning_frame, exc)
            return []

    def _ensure_start_at_first_waypoint(self, waypoints: Sequence[PaperWaypoint]) -> bool:
        """Move to the first waypoint with ordinary pose planning if needed.

        Cartesian planning starts at the robot's *current* pose; it does not
        implicitly teleport to the first item in ``waypoints``.  The writing
        path begins at the paper, while the robot starts in the safe ``up``
        pose, so explicitly bridging that gap is required for a reliable demo.
        """
        if self.dry_run or self.group is None or not waypoints:
            return True
        if not self._recover_invalid_joint_state():
            return False
        try:
            poses = self._transform_waypoints([waypoints[0]])
            if not poses:
                self._status("paper-frame TF unavailable; cannot reach first waypoint")
                return False
            target = poses[0]
            self.group.set_pose_target(target)
            ok = bool(self.group.go(wait=True))
            self.group.stop()
            self.group.clear_pose_targets()
            return ok
        except Exception:
            LOG.exception("Could not move to first writing waypoint")
            try:
                self.group.stop()
                self.group.clear_pose_targets()
            except Exception:
                pass
            return False

    def plan_cartesian(self, waypoints: Sequence[PaperWaypoint]) -> Tuple[Any, float]:
        if not waypoints:
            return None, 0.0
        if not self.initialize():
            return None, 0.0
        if not self._recover_invalid_joint_state():
            return None, 0.0
        poses = self._transform_waypoints(waypoints)
        if not poses:
            self._status("paper-frame TF unavailable; Cartesian planning skipped")
            return None, 0.0
        if self.dry_run or self.group is None:
            return {"waypoints": len(poses), "dry_run": True}, 1.0
        eef_step = float(self._get("motion.eef_step", 0.003))
        # Noetic releases differ: some expose (waypoints, eef_step,
        # avoid_collisions), older ones also expose jump_threshold.  Inspect
        # first, then retain a conservative try-chain for vendor builds.
        try:
            signature = inspect.signature(self.group.compute_cartesian_path)
            names = list(signature.parameters.keys())
        except Exception:
            names = []
        try:
            if "jump_threshold" in names:
                result = self.group.compute_cartesian_path(poses, eef_step, 0.0, True)
            else:
                result = self.group.compute_cartesian_path(poses, eef_step, avoid_collisions=True)
        except TypeError:
            try:
                result = self.group.compute_cartesian_path(poses, eef_step, 0.0)
            except TypeError:
                result = self.group.compute_cartesian_path(poses, eef_step)
        if isinstance(result, tuple) and len(result) >= 2:
            return result[0], float(result[1])
        return result, 0.0

    def retime_trajectory(self, trajectory: Any) -> Any:
        if trajectory is None or self.dry_run or self.group is None:
            return trajectory
        trajectory = self._normalize_trajectory(trajectory)
        try:
            state = self.robot.get_current_state() if self.robot else None
            velocity = float(self._get("motion.velocity_scaling", 0.10))
            acceleration = float(self._get("motion.acceleration_scaling", 0.10))
            return self.group.retime_trajectory(state, trajectory, velocity, acceleration)
        except TypeError:
            try:
                return self.group.retime_trajectory(self.robot.get_current_state(), trajectory, velocity_scaling_factor=velocity, acceleration_scaling_factor=acceleration)
            except Exception:
                LOG.debug("retime signature unsupported", exc_info=True)
        except Exception:
            LOG.warning("retime_trajectory failed; using original trajectory", exc_info=True)
        return trajectory

    def execute(self, trajectory: Any) -> bool:
        if trajectory is None:
            return False
        if self.dry_run:
            self._status("dry-run: trajectory with {} waypoints".format(trajectory.get("waypoints", "?")))
            return True
        if self.group is None:
            return False
        if not self._recover_invalid_joint_state():
            return False
        trajectory = self._normalize_trajectory(trajectory)
        try:
            ok = bool(self.group.execute(trajectory, wait=True))
            self.group.stop()
            self.group.clear_pose_targets()
            self._status("trajectory executed" if ok else "trajectory execution failed")
            return ok
        except Exception as exc:
            self._status("trajectory execution error: {}".format(exc))
            LOG.exception("trajectory execution failed")
            return False

    def plan_and_execute(self, waypoints: Sequence[PaperWaypoint]) -> Tuple[bool, float]:
        if not self._ensure_start_at_first_waypoint(waypoints):
            return False, 0.0
        trajectory, fraction = self.plan_cartesian(waypoints)
        required = float(self._get("moveit.cartesian_fraction_required", 0.98))
        if trajectory is None or fraction < required:
            return False, fraction
        trajectory = self.retime_trajectory(trajectory)
        trajectory = self._normalize_trajectory(trajectory)
        return self.execute(trajectory), fraction

    def plan_with_fallback(self, strokes: Sequence[Stroke], builder: TrajectoryBuilder) -> bool:
        """Try sentence, then progressively smaller units for demo safety."""

        if not strokes:
            return False
        whole = builder.build_sentence_path(strokes)
        ok, fraction = self.plan_and_execute(whole)
        if ok:
            return True
        LOG.warning("whole sentence Cartesian fraction %.3f; starting fallback", fraction)
        # A stroke-level fallback is deterministic and also covers one glyph
        # and one-word cases because LayoutEngine has already split strokes.
        for index, stroke in enumerate(strokes):
            waypoints = builder.stroke_to_waypoints(stroke)
            ok, fraction = self.plan_and_execute(waypoints)
            if not ok:
                LOG.error("fallback stroke %d failed (fraction %.3f)", index, fraction)
                return False
        return True


__all__ = ["MoveItExecutor"]
