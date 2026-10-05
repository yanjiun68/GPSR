# Section: Source image identity keeps ROI, debug image, depth, and consumers synchronized.
std_msgs/Header header
# Section: Command revisions let consumers reject results from an older target request.
uint64 command_revision
# Section: The program-owned target text is never replaced by model-generated text.
string target_id
# Section: Public semantic status is always ok, not_found, ambiguous, or uncertain.
string status
# Section: Operational detail reports tracking, model, input, and throttling state separately.
string processing_status
# Section: The ROI is expressed in pixels of the declared source image dimensions.
sensor_msgs/RegionOfInterest roi
float32 confidence
uint32 image_width
uint32 image_height
# Section: These flags distinguish a fresh model acquisition from a lightweight tracked update.
bool model_generated
bool tracking_valid
