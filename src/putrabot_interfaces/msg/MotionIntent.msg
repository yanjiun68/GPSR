std_msgs/Header header

uint8 MODE_HOLD=0
uint8 MODE_ROTATE=1
uint8 MODE_FOLLOW_POSE=2
uint8 MODE_SEARCH=3

uint8 mode
int32 target_track_id
bool translation_allowed
float32 follow_distance_m
float32 speed_limit_mps
float32 rotate_speed_rad_s
float32 desired_yaw_rad
string mode_label
