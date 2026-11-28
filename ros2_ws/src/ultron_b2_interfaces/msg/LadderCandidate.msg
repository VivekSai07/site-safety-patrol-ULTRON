# Map-scoped ladder candidate. Ambiguity is preserved, never collapsed. JSON: ultron.ladder_candidate/v1.
string SCHEMA_VERSION = "ultron.ladder_candidate/v1"
string ASSOCIATION_NEW = "NEW"
string ASSOCIATION_MATCHED = "MATCHED"
string ASSOCIATION_AMBIGUOUS = "AMBIGUOUS"
string ASSOCIATION_MISSING = "MISSING"

string schema_version
std_msgs/Header header                  # map frame
string candidate_id
string map_id
int64 map_version
geometry_msgs/Point position
float64 yaw
float64[9] position_covariance
string ladder_type
string association_state
string asset_id                         # empty when not yet associated
string[] ambiguous_with
uint32 observation_count
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
string[] detection_ids
bool from_oracle
