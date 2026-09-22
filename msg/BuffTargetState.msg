std_msgs/Header header
bool tracking
bool prediction_ready
bool camera_info_ready
bool tf_ready
bool pnp_ready
string tracker_state
string source
string color
string mode

# Observed state at header.stamp. On TF failure: camera optical frame, tf_ready=false.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Future aim point at header.stamp + prediction_horizon, already predicted ONCE.
geometry_msgs/Point predicted_position
float64 prediction_horizon
float64 depth_age

# Camera-frame state kept for debugging and fallback consumers.
geometry_msgs/Point camera_position
geometry_msgs/Vector3 camera_velocity

# Radians: with tf_ready, target_frame must use x forward/y left/z up,
# yaw=atan2(y,x), pitch=atan2(z,hypot(x,y)). Otherwise optical-relative
# yaw points right/pitch up; consumers MUST NOT treat these as absolute commands.
float64 yaw
float64 pitch
float64 v_yaw
float64 v_pitch
float64 predicted_yaw
float64 predicted_pitch

# Energy mechanism phase state in radians and radians/s.
float64 phase
float64 phase_velocity
float64 predicted_phase

# Frontend diagnostics.
geometry_msgs/Point r_center_px
geometry_msgs/Point fan_center_px
geometry_msgs/Point aim_point_px
float64 radius_px
float64 target_distance
float64 confidence
int32 class_id
