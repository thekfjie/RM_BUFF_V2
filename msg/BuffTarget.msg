std_msgs/Header header
bool tracking
bool prediction_ready
string color
string mode

# Pixel-space target state from the buff tracker.
geometry_msgs/Point r_center_px
geometry_msgs/Point fan_center_px
geometry_msgs/Point aim_point_px

float64 radius_px
float64 observed_angle
float64 raw_angle
float64 delta_angle
float64 compensated_delta
float64 angular_velocity
