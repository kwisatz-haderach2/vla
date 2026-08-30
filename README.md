# vla

UR5e intelligent writing simulation workspace for ROS 1 Noetic and Gazebo 11.

The workspace contains:

- `src/vla_writing`: UR5e writing scene, gripper, pen, table, paper and camera.
- `src/ros2_force_control_interface`: ROS 2 Humble force-control interface for parallel development.
- `src/universal_robot`: official ROS-Industrial dependency, tracked as a Git submodule in the published repository.

Clone and build:

```bash
git clone --recurse-submodules git@github.com:kwisatz-haderach2/vla.git vla_ws
cd vla_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
roslaunch vla_writing writing.launch
```

`writing.launch` starts Gazebo, the UR5e scene, MoveIt and the writing
service.  Use `roslaunch vla_writing writing_scene.launch` when you only need
the simulation scene without MoveIt or the text/image service.
