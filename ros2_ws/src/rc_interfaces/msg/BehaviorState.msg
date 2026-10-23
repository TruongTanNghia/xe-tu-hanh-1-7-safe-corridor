uint8 STATE_LANE_FOLLOW=0
uint8 STATE_AVOID=1
uint8 STATE_FOLLOW=2
uint8 STATE_STOP=3
uint8 STATE_ESTOP=4

std_msgs/Header header
uint8 state
string state_name
string reason
float32 v_ref                  # m/s
float32 time_in_state_s
float32 lead_gap_m             # khoảng cách tới xe/vật phía trước (FOLLOW), <0 nếu không có
float32 lead_speed_mps
uint8 avoid_side               # Corridor.SIDE_*
