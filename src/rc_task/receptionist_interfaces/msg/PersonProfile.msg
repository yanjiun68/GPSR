string profile_id
string role
string name
int32 age
string gender
string favorite_drink
string hometown
receptionist_interfaces/GuestAppearance appearance
bool is_confirmed
float32 confidence_score
builtin_interfaces/Time stamp

# Phase 2 extension fields: face registration metadata plus fallback clues.
float32[] face_embedding
builtin_interfaces/Time register_time
string seat_position
string clothes_color
