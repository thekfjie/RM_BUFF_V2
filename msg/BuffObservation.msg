std_msgs/Header header
bool tracking
bool prediction_ready
bool camera_info_ready
bool pnp_ready
string source
string color
string mode

# Pixel-space debug outputs from the detector/predictor frontend.
geometry_msgs/Point r_center_px
geometry_msgs/Point fan_center_px
geometry_msgs/Point aim_point_px
float64 radius_px
float64 confidence
int32 class_id

# Camera optical-frame angular/space observation.
# camera_position is the predicted/compensated aim point. When PnP is ready,
# camera_pose keeps the current measured blade pose for diagnostics.
geometry_msgs/Vector3 aim_ray
geometry_msgs/Point camera_position
geometry_msgs/Pose camera_pose
float64 yaw
float64 pitch
float64 target_distance

# Energy mechanism phase state.
float64 phase
float64 raw_phase
float64 phase_delta
float64 compensated_phase_delta
float64 phase_velocity
