# Mirrors JSON contract ultron.localization_state/v1 (src/ultron/autonomy/localization_state.py).
# header.stamp = time the pose refers to (scan stamp); header.frame_id = map frame.
string SCHEMA_VERSION = "ultron.localization_state/v1"
string STATE_INITIALIZING = "INITIALIZING"
string STATE_VALID = "VALID"
string STATE_DEGRADED = "DEGRADED"
string STATE_LOST = "LOST"
string STATE_STALE = "STALE"

string schema_version
std_msgs/Header header
builtin_interfaces/Time evaluated_at
string map_id
int64 map_version
string state
string reason
geometry_msgs/Pose pose                 # position (m) + orientation in header.frame_id
bool has_match_score                    # false <=> JSON match_score null
float64 match_score
bool has_covariance                     # false <=> JSON covariance_xy_yaw null
float64[] covariance_xy_yaw
string covariance_source
float64 age_s
float64 age_limit_s
string clock_id
string child_frame_id
string source
bool ground_truth_used                  # must stay false; ground truth never feeds the estimator
