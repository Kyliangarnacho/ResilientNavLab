std_msgs/Header header
uint8 state
bool navigation_allowed
float32 confidence
float32 hard_condition_elapsed_sec
string[] reasons

uint8 STARTING=0
uint8 NAVIGATE=1
uint8 DEGRADED=2
uint8 HOLD=3
