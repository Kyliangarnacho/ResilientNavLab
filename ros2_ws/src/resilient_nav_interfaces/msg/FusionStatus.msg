std_msgs/Header header
string chain_id
uint8 state
string[] accepted_measurements
string[] rejected_measurements
string[] reasons
float32 confidence
builtin_interfaces/Time window_start
builtin_interfaces/Time window_end
uint32 sample_count

uint8 UNKNOWN=0
uint8 NOMINAL=1
uint8 DEGRADED=2
uint8 HOLD=3
