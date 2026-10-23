# Calibration artifacts

Không chỉnh số “cho chạy được” trực tiếp trong source. Mỗi lần calibration phải
lưu rosbag/CSV/report trong `evidence/`, sau đó mới cập nhật YAML runtime được
liệt kê tại `../vehicle_1_7.yaml`.

Các calibration còn bắt buộc trước khi chạy tự hành:

- đo pose `base_link → laser` và `base_link → imu_link`;
- xác nhận quay CCW cho `gyro.z > 0`;
- đo PWM lái trái/giữa/phải khi motor mất điện;
- đo step response ESC `1580, 1585, …, 1600 µs` khi bánh nâng;
- fit và kiểm tra held-out model dọc NMPC.

