# header.stamp: IMU sample / predicted orientation time; frame_id: input IMU frame.
# bias_stamp: posterior time of the directly read bg/ba (not an inverse estimate).
std_msgs/Header header
builtin_interfaces/Time bias_stamp
string world_frame
bool valid
bool zupt_active
uint64 zupt_updates
float64[3] gyro_bias
float64[3] accel_bias
float64[3] raw_angular_velocity
float64[3] raw_linear_acceleration
float64[3] corrected_angular_velocity
# Intrinsic/bias corrected acceleration rotated into world_frame, gravity removed.
float64[3] world_linear_acceleration
