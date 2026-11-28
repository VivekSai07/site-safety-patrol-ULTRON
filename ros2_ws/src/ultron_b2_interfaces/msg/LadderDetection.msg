# One image-space ladder detection. JSON: ultron.ladder_detection/v1.
# backend "oracle_sim" is simulation ground truth and must be labelled ORACLE, never learned perception.
string SCHEMA_VERSION = "ultron.ladder_detection/v1"
string BACKEND_ORACLE_SIM = "oracle_sim"
string BACKEND_YOLO = "yolo"
string BACKEND_OWLV2 = "owlv2"

string schema_version
std_msgs/Header header                  # image stamp and optical frame
string detection_id
string backend
bool is_oracle
string label
float32 confidence                      # backend score, uncalibrated; not a safe-use probability
float32 bbox_x_min                      # pixels
float32 bbox_y_min
float32 bbox_x_max
float32 bbox_y_max
uint32 image_width
uint32 image_height
string evidence_ref                     # local artifact reference (hash/path), never image data
