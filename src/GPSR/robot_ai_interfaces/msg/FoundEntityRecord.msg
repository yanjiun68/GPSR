# Section: Compact RAM-only record shared by GPSR find and downstream skills.
string entity_id
string entity_type
string target_key
geometry_msgs/PoseStamped entity_pose
geometry_msgs/PoseStamped observation_pose
string semantic_location
string viewpoint
string metadata_json
uint64 revision
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
