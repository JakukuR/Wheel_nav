# Local plane in header.frame_id: normal dot point + offset_m = 0.
# has_estimate may be true while valid is false; candidates are not measurements.
std_msgs/Header header
bool has_estimate
bool valid
string method
string reason
string timing_basis
geometry_msgs/Vector3 normal
float64 offset_m
float64 vertical_height_m
float64[2] wheel_heights_m
uint8[2] contact_state
bool imu_axes_verified
bool timing_verified
float64 input_age_s
float64 imu_bracket_span_s
float64 fit_residual_m
string[] sensor_names
geometry_msgs/Vector3[] sensor_ground_normals
float64[] sensor_ground_offsets_m
