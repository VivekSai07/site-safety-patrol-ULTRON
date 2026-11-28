# Status of the common motion_gate (sole publisher of cmd_vel). JSON: ultron.motion_gate_status/v1.
string SCHEMA_VERSION = "ultron.motion_gate_status/v1"

string schema_version
std_msgs/Header header
bool open                               # true only when the last command was forwarded unchanged
string inhibited_reason                 # empty when open; e.g. OPERATOR_ENABLE_STALE, LOCALIZATION: ..., HOLD
string active_source                    # hold | teleop | autonomy | none
string map_id
int64 map_version
string localization_state
bool operator_enabled
float64 operator_enable_age_s
float64 command_age_s
bool hold
bool teleop_requires_localization       # PROPOSED default true (fail-closed)
geometry_msgs/Twist cmd_in
geometry_msgs/Twist cmd_out
