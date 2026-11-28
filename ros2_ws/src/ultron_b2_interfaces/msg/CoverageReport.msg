# Mission-level coverage. Never inferred from waypoint completion; separate from any ladder verdict.
# JSON: ultron.coverage_report/v1.
string SCHEMA_VERSION = "ultron.coverage_report/v1"

string schema_version
std_msgs/Header header                  # map frame
string mission_id
string zone_id
string map_id
int64 map_version
float64 zone_area_m2
float64 observed_area_m2
float64 unknown_area_m2
float64 unreachable_area_m2
float64 coverage_fraction
string[] gap_ids
geometry_msgs/Polygon[] gap_polygons
bool complete
string reason
