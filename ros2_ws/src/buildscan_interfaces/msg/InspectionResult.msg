# BuildScan Rover - Custom Inspection Result Message
# Published by: crack_detection_node
# Subscribed by: inspection_manager, buildscan_dashboard

std_msgs/Header header

# Detection results
int32 cracks_detected
float32 average_confidence
float32 maximum_width
float32 total_length

# Classification
string severity           # NONE / LOW / MEDIUM / HIGH
string recommendation     # Repair recommendation text

# Image paths (on filesystem)
string annotated_image_path
string original_image_path

# Model info
string model_name
string model_version
