std_msgs/Header header
uint8 state
bool localization_usable
float32 confidence
float32 pose_age_sec
float32 map_to_odom_age_sec
float32 scan_health_age_sec
float64 position_variance
float64 yaw_variance
float64 position_jump_m
float64 yaw_jump_rad
string[] reasons

uint8 LOCALIZATION_PROVISIONAL=0
uint8 LOCALIZATION_OK=1
uint8 LOCALIZATION_DEGRADED=2
uint8 LOCALIZATION_LOST=3
