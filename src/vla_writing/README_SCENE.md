# UR5e writing scene (ROS 1 Noetic / Ubuntu 20.04)

This directory contains the simulation scene used by the English, Chinese and
OpenCV image writing demos.  The arm is the **UR5e** model from the
ROS-Industrial `universal_robot` `noetic-devel` repository; the non-e-series
variant is not used.  The repository is kept under `src/universal_robot` so a fresh
checkout does not depend on a system-wide `ur_description` installation.

## Start the complete scene

```bash
cd /home/lyy/vla_ws
source /opt/ros/noetic/setup.bash
catkin build vla_writing
source devel/setup.bash
roslaunch vla_writing writing_scene.launch
```

The launch starts Gazebo, `robot_state_publisher`, the UR5e ros-control
position trajectory controller, paper/table models, and a downward-facing
camera.  The camera publishes:

```text
/vla/paper_camera/image_raw
/vla/paper_camera/camera_info
```

Set `enable_rviz:=true` to open a matching RViz view.  `simulation.launch` and
`demo.launch` are short aliases for video/demo scripts.

`move_group_writing.launch` overlays `config/ur5e_writing.srdf` on the upstream
UR5e planning context.  Its `manipulator` chain ends at `pen_tip` (not
`tool0`), while retaining the six actuated arm joints.  This is important for
Cartesian targets: the fixed 205-mm pen offset is then included by MoveIt.

## Coordinate contract

`paper_frame` is a static frame whose origin is the **top-left** of the sheet:

```text
world -> paper_frame = (0.180, 0.200, 0.517) m
paper width  = 0.420 m (+X/right)
paper height = 0.297 m (logical +v is represented by -Y)
```

The `pen_tip` frame is exported by the end-effector macro.  In the official
UR5e `up` pose the physical pen barrel follows `tool0` +Y (the cylinder is
rotated accordingly), so the nib points down toward the horizontal sheet.  A
writing planner should target `pen_tip` and use `x_p=u`, `y_p=-v` in
`paper_frame`.

## Force-control hand-off (ROS 1 <-> ROS 2)

The scene only loads the bridge parameters; it never launches a ROS 2 process
from `roslaunch`.  The teammate can run the ROS 2 gateway in a second terminal
and connect it with `ros1_bridge`:

```bash
# ROS 2 terminal (Ubuntu 22.04)
source /opt/ros/humble/setup.bash
source <force-workspace>/install/setup.bash
ros2 launch ros2_force_control_interface force_interface.launch.py

# bridge terminal (choose the bridge installation available on that machine)
ros2 run ros1_bridge dynamic_bridge
```

The stable topic names are listed in `config/force_bridge.yaml` and mirrored by
the ROS 2 package in `src/ros2_force_control_interface` (which has
`CATKIN_IGNORE` so ROS 1 catkin does not try to build it):

| Purpose | Topic |
| --- | --- |
| target wrench | `/vla/force_target_wrench` |
| measured wrench | `/vla/measured_wrench` |
| contact state | `/vla/contact_state` and ROS 1 mirror `/vla/pen_contact` |
| enable/mode | `/vla/force_enable`, `/vla/force_mode` |
| pen frame | `pen_tip`; paper normal is +Z in `paper_frame` |

The default ROS 1 scene remains position/trajectory controlled, so the
force-control branch can be enabled or replaced without changing the URDF or
the `List[Stroke]` trajectory interface.

## Assets

* `urdf/writing_scene.urdf.xacro` — official UR5e Gazebo macro plus gripper and
  pen (`pen_tip` frame).
* `models/writing_table` — static table.
* `models/writing_paper` — 420 x 297 mm sheet and the `ink_visual_plugin`
  anchor (`ink_anchor_visual`).
* `models/paper_camera` — 800 x 600 Gazebo camera aimed at the sheet.
* `worlds/writing.world` — lighting, ground and model placement.
