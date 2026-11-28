# Operator motion-enable heartbeat. A missing or stale heartbeat closes the motion gate.
# JSON: ultron.operator_enable/v1.
string SCHEMA_VERSION = "ultron.operator_enable/v1"

string schema_version
std_msgs/Header header
bool enabled
string operator_id
string reason
uint32 sequence
