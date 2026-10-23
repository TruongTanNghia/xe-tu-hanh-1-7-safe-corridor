uint8 STATUS_INVALID=0
uint8 STATUS_DETECTED=1
uint8 STATUS_STALE=2

std_msgs/Header header
float64[] s_m
float64[] curvature_inv_m
float64 cte_m
float64 heading_error_rad
float64 confidence
uint8 status
# Bổ sung cho Safe Corridor (không ảnh hưởng NMPC cũ):
# khoảng cách dọc (m) từ base_link tới điểm s_m[0] trên tâm làn và
# độ lệch ngang (m, +trái) của từng điểm tâm làn tương ứng với s_m.
float64 evaluation_forward_m
float64[] path_lateral_m
