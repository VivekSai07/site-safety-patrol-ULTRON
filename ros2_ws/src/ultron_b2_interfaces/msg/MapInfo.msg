# Saved/loaded map record (2D slam_toolbox map + optional 3D map). JSON: ultron.map_info/v1.
string SCHEMA_VERSION = "ultron.map_info/v1"

string schema_version
std_msgs/Header header
string map_id
int64 map_version
string yaml_path
string image_sha256
string posegraph_path
string map3d_path
string map3d_sha256
float64 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
float64 free_thresh
float64 occupied_thresh
builtin_interfaces/Time created_at
string source
