# Append-only mission event. Arrival is never an inspection result. JSON: ultron.mission_event/v1.
string SCHEMA_VERSION = "ultron.mission_event/v1"

string schema_version
std_msgs/Header header
string mission_id
string event_id
string event_type
string from_state
string to_state
string reason
string asset_id
string goal_id
string payload_json                     # extra structured data as JSON text
