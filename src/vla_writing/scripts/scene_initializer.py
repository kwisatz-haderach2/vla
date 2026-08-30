#!/usr/bin/env python3
"""Hold the UR5e in its requested startup pose before unpausing Gazebo.

``gazebo_ros/spawn_model`` accepts ``-J`` values, but the trajectory
controller is normally started at the same time and initially commands zero
for every joint.  This small ROS 1 helper makes startup deterministic:

1. wait for Gazebo to report the spawned model and for the configuration
   service;
2. set the six joint positions while physics is paused;
3. unpause physics so Gazebo 11 can service the controller switch request;
4. start the selected trajectory controller and publish a one-point hold
   trajectory at the same positions.

The result is a real, bounded Gazebo state that MoveIt can immediately use.
The same node works for the position controller used by the demo and for the
effort trajectory controller used by the optional force-control hand-off.
"""

from __future__ import print_function

import time

import rospy

from controller_manager_msgs.srv import ListControllers, SwitchController
from gazebo_msgs.srv import GetWorldProperties, SetModelConfiguration
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Pen tip is approximately 54 mm above the paper in this startup pose.  Keep
# this fallback identical to writing_scene.launch so running the helper by
# hand cannot silently select a different wrist configuration.
DEFAULT_JOINT_POSITIONS = [0.05400, -1.00600, 2.26440, 1.88330, -0.05400, 0.00160]


def _as_float_list(value, fallback):
    try:
        values = [float(item) for item in value]
        return values if values else list(fallback)
    except (TypeError, ValueError):
        return list(fallback)


def _controller_state(controller_name):
    """Return the state string for a loaded controller, or ``None``."""

    service = rospy.get_param("~list_controllers_service", "/controller_manager/list_controllers")
    try:
        rospy.wait_for_service(service, timeout=2.0)
        response = rospy.ServiceProxy(service, ListControllers)()
        for item in getattr(response, "controller", []):
            if str(getattr(item, "name", "")) == controller_name:
                return str(getattr(item, "state", ""))
    except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException):
        pass
    return None


def _switch_start(controller_name):
    service = rospy.get_param("~switch_controller_service", "/controller_manager/switch_controller")
    try:
        rospy.wait_for_service(service, timeout=30.0)
        switch = rospy.ServiceProxy(service, SwitchController)
        deadline = time.time() + 30.0
        while not rospy.is_shutdown():
            try:
                response = switch(
                    start_controllers=[controller_name],
                    stop_controllers=[],
                    strictness=SwitchControllerRequest_BEST_EFFORT,
                    start_asap=True,
                    timeout=2.0,
                )
            except TypeError:
                # Older generated service bindings accept positional fields only.
                response = switch([controller_name], [], SwitchControllerRequest_BEST_EFFORT, True, 2.0)
            if bool(getattr(response, "ok", True)):
                return True
            if time.time() >= deadline:
                rospy.logerr("Could not start %s: %s", controller_name, response)
                return False
            time.sleep(0.2)
    except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as exc:
        rospy.logerr("Could not start %s: %s", controller_name, exc)
        return False


# ``SwitchController`` constants are not exposed consistently by all Noetic
# generated Python modules.  The service definition uses 1 for BEST_EFFORT.
SwitchControllerRequest_BEST_EFFORT = 1


def main():
    rospy.init_node("scene_initializer")

    model_name = str(rospy.get_param("~model_name", "ur5e_writing"))
    urdf_param = str(rospy.get_param("~urdf_param_name", "robot_description"))
    controller_name = str(rospy.get_param("~trajectory_controller", "pos_joint_traj_controller"))
    joint_names = [str(item) for item in rospy.get_param("~joint_names", DEFAULT_JOINT_NAMES)]
    joint_positions = _as_float_list(
        rospy.get_param("~joint_positions", DEFAULT_JOINT_POSITIONS), DEFAULT_JOINT_POSITIONS)
    if len(joint_positions) != len(joint_names):
        rospy.logfatal("joint_names and joint_positions must have equal length")
        return

    # Do not call SetModelConfiguration while spawn_model is still inserting
    # the URDF.  Gazebo serializes those requests; racing the insertion can
    # leave the Python service proxy blocked indefinitely even though the
    # model appears a few milliseconds later.  GetWorldProperties is safe to
    # poll while physics is paused and gives us a deterministic barrier.
    world_props_name = str(rospy.get_param("~world_properties_service", "/gazebo/get_world_properties"))
    try:
        rospy.wait_for_service(world_props_name, timeout=30.0)
        get_world = rospy.ServiceProxy(world_props_name, GetWorldProperties)
        deadline = time.time() + 60.0
        while not rospy.is_shutdown():
            response = get_world()
            if model_name in list(getattr(response, "model_names", [])):
                break
            if time.time() >= deadline:
                rospy.logfatal("Gazebo never spawned model %s", model_name)
                return
            time.sleep(0.1)
    except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as exc:
        rospy.logfatal("Could not wait for Gazebo model %s: %s", model_name, exc)
        return

    set_config_name = str(rospy.get_param("~set_model_configuration_service", "/gazebo/set_model_configuration"))
    try:
        rospy.wait_for_service(set_config_name, timeout=30.0)
        set_config = rospy.ServiceProxy(set_config_name, SetModelConfiguration)
        # The Gazebo service is advertised before spawn_model has finished
        # inserting the URDF.  Retry on the transient "model does not exist"
        # response instead of exiting and leaving the controller stopped.
        deadline = time.time() + 60.0
        while not rospy.is_shutdown():
            try:
                response = set_config(
                    model_name=model_name,
                    urdf_param_name=urdf_param,
                    joint_names=joint_names,
                    joint_positions=joint_positions,
                )
            except TypeError:
                response = set_config(model_name, urdf_param, joint_names, joint_positions)
            if not hasattr(response, "success") or response.success:
                break
            if time.time() >= deadline:
                rospy.logfatal("Gazebo rejected the initial joint configuration: %s", response)
                return
            rospy.logwarn_throttle(5.0, "Waiting for spawned model %s: %s", model_name, response)
            time.sleep(0.2)
    except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as exc:
        rospy.logfatal("Could not set initial Gazebo configuration: %s", exc)
        return

    # controller_manager's switch service is serviced from Gazebo's update
    # thread.  On Gazebo 11 it may not answer while the world is paused.  The
    # model is already in the requested pose, so briefly unpause first, start
    # the controller, and immediately send a hold trajectory.  This removes
    # the old deadlock while keeping the transient motion bounded.
    unpause_name = str(rospy.get_param("~unpause_service", "/gazebo/unpause_physics"))
    should_unpause = bool(rospy.get_param("~unpause", True))
    if should_unpause:
        try:
            rospy.wait_for_service(unpause_name, timeout=10.0)
            rospy.ServiceProxy(unpause_name, Empty)()
        except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as exc:
            rospy.logwarn("Gazebo could not be unpaused before controller start: %s", exc)

    # The --stopped controller spawner can still be parsing/loading its
    # controller immediately after the Gazebo model exists.  Wait until the
    # controller manager reports it as loaded; otherwise switch_controller may
    # return success for an empty request and the arm remains uncontrolled.
    controller_deadline = time.time() + 30.0
    state = None
    while not rospy.is_shutdown():
        state = _controller_state(controller_name)
        if state in ("initialized", "stopped", "running"):
            break
        if time.time() >= controller_deadline:
            rospy.logfatal("Controller %s was not loaded before startup", controller_name)
            return
        time.sleep(0.1)

    if state != "running" and not _switch_start(controller_name):
        return

    topic = str(rospy.get_param("~command_topic", "/%s/command" % controller_name))
    publisher = rospy.Publisher(topic, JointTrajectory, queue_size=1, latch=True)
    # Give the controller one scheduler cycle to advertise its command topic.
    time.sleep(0.2)
    message = JointTrajectory()
    message.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(joint_positions)
    point.velocities = [0.0] * len(joint_names)
    point.time_from_start = rospy.Duration(float(rospy.get_param("~hold_duration", 1.0)))
    message.points = [point]
    # Publish a few copies so a late subscriber cannot miss the latched hold.
    for _ in range(5):
        publisher.publish(message)
        time.sleep(0.05)

    if not should_unpause:
        rospy.loginfo("Gazebo remains paused; start %s manually before publishing trajectories", controller_name)

    rospy.loginfo("UR5e startup pose initialized and %s is holding it", controller_name)


if __name__ == "__main__":
    main()
