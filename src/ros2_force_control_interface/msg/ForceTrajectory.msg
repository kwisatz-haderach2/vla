# Optional force-aware Cartesian trajectory contract.
# Waypoints are expressed in header.frame_id and should preserve a fixed pen orientation.
std_msgs/Header header
geometry_msgs/Pose[] waypoints
float64 eef_step
float64 target_normal_force
float64 force_tolerance
float64 max_normal_velocity
bool stop_on_contact_loss
string source
