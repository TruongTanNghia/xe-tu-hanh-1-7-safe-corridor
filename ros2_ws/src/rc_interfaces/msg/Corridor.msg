# Hành lang an toàn trong hệ Frenet (s, d) bám theo tâm làn.
# s = 0 tại hình chiếu của base_link lên đường tham chiếu; d dương về bên trái.

uint8 SIDE_CENTER=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

std_msgs/Header header
bool lane_valid
float32 ds                     # m, bước rời rạc hoá theo s
float32[] s                    # trạm s_i
float32[] lower                # l(s_i): biên dưới cho tâm xe (đã trừ w/2 + lề)
float32[] upper                # u(s_i)
float32[] d_ref                # tham chiếu ngang cho NMPC
float32[] v_max                # m/s, giới hạn tốc độ theo trạm
float32[] kappa_ref            # 1/m, độ cong đường tham chiếu tại s_i
float32 ego_d                  # m, vị trí ngang hiện tại của xe
float32 ego_heading_error      # rad
uint8 side
bool feasible
float32 blocked_at_s           # m, <0 nếu không bị chặn trong tầm nhìn
float32 min_free_width         # m, bề rộng hẹp nhất trong hành lang khả thi
uint8 behavior_state           # trùng BehaviorState.state
float32 v_ref                  # m/s, tốc độ mục tiêu do FSM cấp
uint32 obstacle_count          # số vật cản ảnh hưởng tới hành lang
