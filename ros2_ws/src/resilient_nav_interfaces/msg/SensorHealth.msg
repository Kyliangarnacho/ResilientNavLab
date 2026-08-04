std_msgs/Header header
string sensor
string source_topic
uint8 state
float32 health_score
float32 confidence
string detected_fault
string[] reasons
string[] metric_names
float64[] metric_values
builtin_interfaces/Time window_start
builtin_interfaces/Time window_end
uint32 sample_count

uint8 UNKNOWN=0
uint8 HEALTHY=1
uint8 DEGRADED=2
uint8 FAULT=3
