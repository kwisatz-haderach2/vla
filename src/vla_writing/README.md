# VLA Writing (ROS 1 Noetic)

This package provides the UR5e/Gazebo writing scene and the common English,
Hanzi and OpenCV trajectory pipeline.  Start the complete simulation with:

```bash
source /opt/ros/noetic/setup.bash
source /home/lyy/vla_ws/devel/setup.bash
roslaunch vla_writing writing.launch
```

Scene details, coordinate frames, camera topics, and the ROS 1↔ROS 2
force-control hand-off are documented in [README_SCENE.md](README_SCENE.md).
