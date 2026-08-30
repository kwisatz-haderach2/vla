# Measured and commanded force state.
# normal_force and target_normal_force are positive magnitudes along the paper normal.
std_msgs/Header header
geometry_msgs/Wrench measured_wrench
geometry_msgs/Wrench target_wrench
float64 normal_force
float64 target_normal_force
float64 normal_force_error
float64 force_tolerance
float64 contact_threshold
float64 contact_depth
bool in_contact
bool controller_enabled
uint8 mode
string controller_state
string source
