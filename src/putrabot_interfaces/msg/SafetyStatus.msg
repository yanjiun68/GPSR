std_msgs/Header header

uint8 HEALTH_OK=0
uint8 HEALTH_WAITING=1
uint8 HEALTH_INHIBITED=2
uint8 HEALTH_FAULT=3

uint8 STOP_NONE=0
uint8 STOP_DISABLED_BY_SERVER=1
uint8 STOP_TRACKER_HEARTBEAT_TIMEOUT=2
uint8 STOP_TF_UNAVAILABLE_TIMEOUT=3
uint8 STOP_NAV2_PENDING=4
uint8 STOP_NAV2_UNAVAILABLE=5
uint8 STOP_OBSTACLE_INHIBIT=6
uint8 STOP_OBSTACLE_INHIBIT_TIMEOUT=7
uint8 STOP_WAITING_FOR_TARGET=8
uint8 STOP_SELECTED_TARGET_PENDING=9
uint8 STOP_SELECTED_TARGET_STALE_TIMEOUT=10
uint8 STOP_BEHAVIOR_ABORTED=11

uint8 SOURCE_NONE=0
uint8 SOURCE_SERVER=1
uint8 SOURCE_TRACKER=2
uint8 SOURCE_TF=3
uint8 SOURCE_NAVIGATION=4
uint8 SOURCE_SAFETY=5
uint8 SOURCE_TARGET_LOCK=6
uint8 SOURCE_BEHAVIOR=7

uint8 health_state
uint8 stop_reason_code
uint8 stop_source_code
bool inhibit_active
bool follow_enabled
bool target_valid
bool nav_available
bool tf_healthy
bool obstacle_inhibit
float32 tracker_age_sec
float32 target_age_sec
float32 tf_age_sec
string stop_reason
string stop_source
string health_label
