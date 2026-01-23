std_msgs/Header header

uint8 state
string target_id
string detail

uint8 READY=0
uint8 MOVING_TO_TARGET=1
uint8 ARRIVED=2
uint8 WAITING_FOR_SPEECH=3
uint8 RETURNING_TO_START=4
uint8 AT_START=5
uint8 EMERGENCY_STOP=6
uint8 FAILED=7

