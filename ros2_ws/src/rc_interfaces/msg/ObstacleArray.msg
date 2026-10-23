std_msgs/Header header
string tracking_frame          # frame dùng để chạy Kalman (odom hoặc base_link)
uint32 scan_point_count        # số điểm LiDAR còn lại sau ROI/lane gate
uint32 cluster_count
float32 processing_time_ms
Obstacle[] obstacles
