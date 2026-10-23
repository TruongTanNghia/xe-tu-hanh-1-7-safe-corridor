# Một vật cản đã được bám vết (track) từ LiDAR 2D.
# Toạ độ nằm trong header.frame_id của ObstacleArray (mặc định base_link).

uint8 SHAPE_CIRCLE=0
uint8 SHAPE_BOX=1

uint32 id
float32 x                      # m, tâm vật cản
float32 y                      # m, +y là bên trái xe
float32 yaw                    # rad, hướng cạnh dài (chỉ dùng cho SHAPE_BOX)
float32 length                 # m, cạnh dài (box) hoặc đường kính (circle)
float32 width                  # m, cạnh ngắn (box) hoặc đường kính (circle)
float32 radius                 # m, bán kính bao ngoài tính từ tâm
float32 vx                     # m/s, vận tốc ước lượng trong header frame
float32 vy
float32[4] position_covariance # 2x2 [xx, xy, yx, yy]
bool is_dynamic
bool confirmed
float32 confidence             # 0..1
uint8 shape
uint32 point_count
uint32 age_frames
