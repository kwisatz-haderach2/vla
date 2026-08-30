# High-level command sent by a writing planner or force controller.
# The paper normal is +Z in the selected wrench frame. If target_normal_force
# is greater than zero it is authoritative; otherwise desired_wrench.force.z
# is used as a positive magnitude.
std_msgs/Header header
geometry_msgs/Wrench desired_wrench
float64 target_normal_force
float64 force_tolerance
float64 max_normal_velocity
bool enable
bool reset_integrator
string controller_mode
string source
