# Section: Canonical executable primitive identifiers shared by every ActionQueue producer.
string NAVIGATE=NAVIGATE
string FIND_OBJECT=FIND_OBJECT
string FIND_PERSON=FIND_PERSON
string FIND_PERSON_BY_ATTRIBUTE=FIND_PERSON_BY_ATTRIBUTE
string COUNT_OBJECTS=COUNT_OBJECTS
string COUNT_PEOPLE=COUNT_PEOPLE
string EXTRACT_PERSON_INFO=EXTRACT_PERSON_INFO
string PICK_OBJECT=PICK_OBJECT
string PLACE_OBJECT=PLACE_OBJECT
string HANDOVER_OBJECT=HANDOVER_OBJECT
string FOLLOW_PERSON=FOLLOW_PERSON
string GUIDE_PERSON=GUIDE_PERSON
string SAY=SAY
string ASK_AND_LISTEN=ASK_AND_LISTEN
string FIXED_INFORMATION=FIXED_INFORMATION

# Section: One ordered executable primitive; surface verbs never enter action_type.
string step_id
string action_type
string task_id
string task_family
string target_object
string target_person
string target_location
string target_category
string parameters_json
float32 timeout_sec
uint32 max_retries
string[] requires_success
string[] requires_data
string result_variable
