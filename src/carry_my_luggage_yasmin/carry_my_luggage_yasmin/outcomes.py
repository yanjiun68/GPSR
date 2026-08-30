"""Section: Shared outcome names used by both YASMIN state machines."""


# Section: State-level outcomes make retry and terminal edges explicit in the viewer.
SUCCEEDED = "succeeded"
RETRY = "retry"
EXHAUSTED = "exhausted"
FAILED = "failed"
CANCELED = "canceled"
YES = "yes"
NO = "no"
POSE_FAILED = "pose_failed"
BAG_FAILED = "bag_failed"

# Section: Machine-level outcomes are returned to the ROS executable.
COMPLETED = "completed"
ABORTED = "aborted"
