# Semantic classification statuses are represented by the documented string constants below.
string STATUS_KNOWN=KNOWN
string STATUS_UNKNOWN=UNKNOWN
string STATUS_LOCALIZATION_UNCERTAIN=LOCALIZATION_UNCERTAIN

# Timestamp and frame identify the metric pose used for this classification.
builtin_interfaces/Time stamp
string frame_id

# Semantic fields are empty when the associated semantic level is unknown.
string status
string room
string location
string nearest_waypoint

# Pose fields support diagnostics without creating another localization source.
bool pose_available
float64 x
float64 y
float64 yaw

# Covariance fields are populated only for the AMCL pose source.
bool covariance_available
float64 max_xy_variance
