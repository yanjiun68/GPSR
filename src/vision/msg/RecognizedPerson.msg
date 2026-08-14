std_msgs/Header header

uint8 UNKNOWN=0
uint8 AMBIGUOUS=1
uint8 RECOGNIZED=2

int8 ATTRIBUTE_UNKNOWN=-1
int8 ATTRIBUTE_FALSE=0
int8 ATTRIBUTE_TRUE=1

uint8 status
string status_text

int32 person_id
int32 track_id

string person_name
uint16 age
string favourite_drink
bool has_registered_profile

int8 wearing_glasses
float32 glasses_confidence

string shirt_colour
float32 shirt_colour_confidence

int8 wearing_face_mask
float32 face_mask_confidence

string hair_colour
float32 hair_colour_confidence

bool is_nearest
uint16 proximity_rank
float32 view_area_ratio
string proximity_method

int32 nearest_track_id
int32 nearest_person_id

bool registration_active
bool is_registration_target
int32 active_registration_person_id
int32 active_registration_track_id

float32 final_score
float32 second_score

float32 body_score
float32 upper_clothing_score
float32 lower_clothing_score
float32 object_score

float32 body_quality
float32 upper_clothing_quality
float32 lower_clothing_quality
float32 object_quality
float32 visual_attribute_quality

float32[4] bbox_xyxy
string visual_summary
