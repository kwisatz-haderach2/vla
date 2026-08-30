# ROS 2 force-control interface

This is the ROS 2 (Humble, Ubuntu 22.04) side of the UR5e writing demo.  It is
kept in `src/ros2_force_control_interface` so the ROS 1 Noetic/catkin package
can remain the owner of Gazebo, MoveIt, text/image parsing, and Cartesian path
execution.  The package is an independent `ament_cmake` package and contains
no ROS 1 code.

The included `force_control_gateway.py` is deliberately hardware agnostic.  It
normalises force commands, estimates contact from a measured wrench, and emits
a stable state stream.  A teammate can put their admittance/impedance or
torque loop behind this boundary without changing the ROS 1 writing planner.
The gateway is a monitor/contract implementation, not a claim of closed-loop
torque control by itself.

## Build on Ubuntu 22.04 / ROS 2 Humble

Use a ROS 2 shell (do not source ROS 1 in that shell):

```bash
source /opt/ros/humble/setup.bash
cd /home/lyy/vla_ws
colcon build --packages-select ros2_force_control_interface --symlink-install
source install/setup.bash
ros2 launch ros2_force_control_interface force_interface.launch.py
```

The workspace also contains a ROS 1 catkin build tree.  `CATKIN_IGNORE` in
this directory makes `catkin build` skip this package.  Do not add a
`COLCON_IGNORE` file here: that would prevent the Humble build.  If a separate
ROS 2 workspace is preferred, copy or symlink this one package into its `src/`
directory.

## Interfaces

### Custom ROS 2 interfaces

| Name | Type | Purpose |
| --- | --- | --- |
| `/vla/force_command` | `ros2_force_control_interface/msg/ForceCommand` | Enable force mode and set the desired normal force. |
| `/vla/measured_wrench` | `geometry_msgs/msg/WrenchStamped` | Measured F/T data from the teammate's sensor/controller. |
| `/vla/force_target_wrench` | `geometry_msgs/msg/WrenchStamped` | Normalised target sent to the low-level force controller. |
| `/vla/force_state` | `ros2_force_control_interface/msg/ForceState` | Measured force, target, error, and mode. |
| `/vla/contact_state` | `ros2_force_control_interface/msg/ContactState` | Contact estimate used by the writing node. |
| `/vla/set_force_control` | `ros2_force_control_interface/srv/SetForceControl` | Change target/enable state synchronously. |
| `/vla/set_compliance` | `ros2_force_control_interface/srv/SetCompliance` | Enable compliance and provide stiffness/damping limits. |
| `/vla/execute_force_writing` | `ros2_force_control_interface/action/ExecuteForceWriting` | Optional force-aware Cartesian waypoint action contract. |

`ForceCommand.target_normal_force` is authoritative when greater than zero;
otherwise `desired_wrench.force.z` is used.  The gateway publishes normal force
as a positive magnitude, so a sensor mounted with the opposite sign does not
change the planner API.  `force_axis` defaults to `z` and should match the
paper normal in the selected wrench frame.

### ROS 1 <-> ROS 2 bridge ABI

The following standard-message topics are intentionally provided for
`ros1_bridge` dynamic bridging.  A ROS 1 Noetic node can use these without
building a ROS 1 copy of the custom interfaces:

| Topic | ROS type | Direction | Contract |
| --- | --- | --- | --- |
| `/vla/pen_pose` | `geometry_msgs/PoseStamped` | ROS 1 -> ROS 2 | Actual pen-tip pose; `header.frame_id` identifies the frame. |
| `/vla/force_cmd_wrench` | `geometry_msgs/WrenchStamped` | ROS 1 -> ROS 2 | Desired normal force in `wrench.force.z`; positive magnitude. |
| `/vla/force_target` | `std_msgs/Float64` | ROS 1 -> ROS 2 | Alternate scalar desired normal force in newtons. |
| `/vla/force_enable` | `std_msgs/Bool` | ROS 1 -> ROS 2 | Start/stop force control. |
| `/vla/force_mode` | `std_msgs/String` | ROS 1 -> ROS 2 | `force`, `compliance`, or `monitor`. |
| `/vla/measured_wrench` | `geometry_msgs/WrenchStamped` | ROS 2 -> gateway | Measured wrench; teammate's controller normally publishes it. |
| `/vla/force_state_wrench` | `geometry_msgs/WrenchStamped` | ROS 2 -> ROS 1 | Mirror of the latest measured wrench. |
| `/vla/pen_contact` | `std_msgs/Bool` | ROS 2 -> ROS 1 | Contact gate for ink rendering (`true` while writing contact exists). |
| `/vla/status` | `std_msgs/String` | ROS 2 -> ROS 1 | `DISABLED`, `FORCE_SEARCH`, `FORCE_CONTACT`, `COMPLIANCE_SEARCH`, etc. |

The gateway subscribes to both the custom command and the standard mirror
topics.  This permits a quick demo with only standard messages; the custom
messages/actions remain available when both sides are built with ROS 2.

`ros1_bridge dynamic_bridge` can automatically bridge the standard messages
listed above.  It cannot invent a ROS 1 definition for `ForceCommand`,
`ForceState`, or the action.  If the custom interfaces are needed across the
boundary, copy the exact `.msg/.srv` definitions into a ROS 1 message package
and build a static bridge after both workspaces are sourced.  For today's
demo, using the standard mirror topics is simpler and keeps the ROS 1 package
independent of this ROS 2 package.

## Running beside the ROS 1 writer

1. In a ROS 1 Noetic shell, launch the UR5e/Gazebo writer and publish the actual
   pen pose on `/vla/pen_pose`.  The ROS 1 node can send the requested force on
   `/vla/force_target` and toggle `/vla/force_enable`.
2. In an Ubuntu 22.04/Humble shell, run `force_interface.launch.py` and the
   teammate's low-level controller.  The controller subscribes to
   `/vla/force_target_wrench`, applies the normal-axis correction, and publishes
   `/vla/measured_wrench`.
3. If the ROS 1 and ROS 2 machines/processes share a network, start
   `ros1_bridge dynamic_bridge` in an environment where both distributions and
   the standard message packages are sourced.  For separate machines, use the
   normal ROS master/DDS bridge setup and verify that `/vla/pen_contact` and
   `/vla/status` are visible on the ROS 1 side.

The action is optional for the first video.  The ROS 1 planner may continue to
execute its existing MoveIt Cartesian path while the ROS 2 controller owns only
the pen's normal-axis compliance.  This separation avoids mixing ROS 1 and ROS
2 client libraries in one process.

## Minimal smoke test (Humble)

```bash
ros2 topic echo /vla/force_state
ros2 topic echo /vla/pen_contact
ros2 topic pub --once /vla/force_target std_msgs/msg/Float64 '{data: 2.0}'
ros2 topic pub --once /vla/force_enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /vla/measured_wrench geometry_msgs/msg/WrenchStamped \
  "{wrench: {force: {z: 2.1}}}"
```

With the default `contact_threshold` of 1 N, the last command should make
`/vla/pen_contact` true and report `FORCE_CONTACT` on `/vla/status`.

## Integration notes for the force-control teammate

- Keep torque/current commands and robot-driver details in the teammate's
  ROS 2 package; this package should remain a transport and state boundary.
- Publish the wrench in a documented frame (normally `paper_frame` or the
  force/torque sensor frame) and set `header.frame_id` consistently.
- Treat `/vla/pen_contact` as an estimate for ink rendering, not as a safety
  stop.  The low-level loop must enforce velocity, displacement, and watchdog
  limits.
- On controller loss, publish zero/disabled state and stop commanding motion.
- The ROS 1 writer can be tested without the force loop by publishing a fake
  `WrenchStamped`; this makes the interface useful before hardware integration.
