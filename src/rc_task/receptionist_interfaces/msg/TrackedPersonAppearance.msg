# Section: One frame-associated appearance observation for a canonical tracked person.
std_msgs/Header header
string tracking_session_id
int32 track_id
int32 person_id
string profile_id
float32[4] bbox_xyxy
receptionist_interfaces/GuestAppearance appearance
receptionist_interfaces/PersonProfile profile
string pose
float32 pose_confidence
string gesture
float32 gesture_confidence
