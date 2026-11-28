# Mission state machine status. JSON: ultron.mission_status/v1.
# supervisor_action mirrors src/ultron/supervisor.py actions.
string SCHEMA_VERSION = "ultron.mission_status/v1"
string STATE_IDLE = "IDLE"
string STATE_PATROL = "PATROL"
string STATE_EXPLORE = "EXPLORE"
string STATE_DETECT = "DETECT"
string STATE_APPROACH = "APPROACH"
string STATE_SETTLE = "SETTLE"
string STATE_CAPTURE = "CAPTURE"
string STATE_ASSESS = "ASSESS"
string STATE_COMMIT = "COMMIT"
string STATE_RETURN_BASE = "RETURN_BASE"
string STATE_HOLD = "HOLD"
string STATE_DONE = "DONE"
string STATE_ABORTED = "ABORTED"
string ACTION_INSPECT = "INSPECT"
string ACTION_SURVEY = "SURVEY"
string ACTION_RETURN_BASE = "RETURN_BASE"
string ACTION_RETURN_BASE_WITH_GAPS = "RETURN_BASE_WITH_GAPS"
string ACTION_AWAIT_OPERATOR = "AWAIT_OPERATOR"

string schema_version
std_msgs/Header header
string mission_id
string state
string supervisor_action
string reason
string active_asset_id
string active_goal_id
string map_id
int64 map_version
bool hold
uint32 ladders_known
uint32 ladders_captured
