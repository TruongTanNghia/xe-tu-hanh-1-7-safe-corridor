# Bước 5 — đo extrinsic LiDAR/IMU và kiểm tra trục gyro

## Trạng thái hiện tại — 2026-07-16

`ros2_ws/src/rc_localization/config/sensor_extrinsics.yaml` hiện có
`status: measured_ccw_verified`. Evidence lịch sử
`evidence/20260629-171034_imu_ccw_axis_retry` PASS cho gyro.z khi quay CCW và
ghi số đo `base_link → laser`, `base_link → imu_link`. Tuy nhiên runtime hiện
đang có 0 publisher `/bno055/imu`; đây là lỗi bringup BNO055 riêng, không được
coi là đã xác nhận live IMU/EKF trong session hiện tại.

Theo quyết định vận hành hiện tại, Gate 5 dùng LiDAR/RF2O → `/odometry/filtered`
và khởi động fusion với `start_imu:=false`; không coi BNO055 là điều kiện của
lượt test này. Điều đó không miễn việc ghi rõ giới hạn sensor trong report.

## 1. Chuẩn bị

Không cấp nguồn động lực và không chạy actuation. Đặt xe trên mặt phẳng, đánh
dấu hệ trục giống mô phỏng:

- gốc `base_link`: trung điểm giữa tâm cầu trước và tâm cầu sau, ở cao độ tâm
  trục bánh;
- `+X`: hướng xe chạy tới;
- `+Y`: bên trái xe;
- `+Z`: hướng lên.

Dùng thước đo từ gốc tới tâm quét LiDAR và tâm chip IMU. Ghi mét, không ghi
centimet. Roll/pitch/yaw dùng radian theo quy tắc bàn tay phải.

Nhập số đo vào
`ros2_ws/src/rc_localization/config/sensor_extrinsics.yaml`, nhưng giữ
`status: blocked_requires_measurement` cho tới khi toàn bộ bước này PASS.

## 2. Lệnh chạy

Build sau khi sửa YAML:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/build.sh rc_localization rc_bringup
bash scripts/run_fusion.sh
```

Launch vẫn cho phép override tạm để thử mà chưa sửa YAML:

```bash
bash scripts/run_fusion.sh \
  lidar_x:=0.10 lidar_y:=0.00 lidar_z:=0.12 lidar_yaw:=0.00 \
  imu_x:=0.00 imu_y:=0.00 imu_z:=0.10 imu_yaw:=0.00
```

Các số trên chỉ minh họa cú pháp, không phải số của xe.

## 3. Quan sát

Terminal 2:

```bash
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 topic echo /bno055/imu --field angular_velocity.z
```

Đặt một hộp cách LiDAR khoảng `0,5 m` ngay trước đầu xe và mở web viewer:

```bash
bash scripts/run_lidar_web_viewer.sh
```

Hộp phải xuất hiện phía `forward` trên hình. Sau đó nâng xe hoặc giữ chắc,
xoay toàn bộ xe ngược chiều kim đồng hồ nhìn từ trên xuống; `gyro.z` phải dương.
Không kết luận từ một mẫu—phải ghi bag.

## 4. Xuất dữ liệu

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario imu_ccw_axis)
bash scripts/evidence/record.sh --session "$SESSION" --duration 15 -- \
  /bno055/imu /scan /tf_static
```

Trong đúng 15 giây đó: giữ yên khoảng 2 giây, quay CCW liên tục ít nhất
`45°`, rồi giữ yên. Lưu thêm snapshot LiDAR:

```bash
bash scripts/save_lidar_snapshot.sh "$SESSION/images/lidar_front_object.svg"
```

Nếu sang terminal mới, lấy lại đúng session bằng:

```bash
SESSION=$(realpath evidence/latest)
```

## 5. Đánh giá

```bash
SESSION=$(realpath evidence/latest)
bash scripts/evidence/evaluate.sh \
  --mode imu-axis \
  --session "$SESSION" \
  --expected-direction ccw
```

PASS khi:

- ít nhất 20 mẫu có `|gyro.z| ≥ 0,05 rad/s`;
- ít nhất 80% mẫu chuyển động có dấu dương;
- góc tích phân dương tối thiểu `0,25 rad`;
- TF khớp số đo và vật phía trước nằm đúng hướng `forward`.

Ngưỡng `0,05 rad/s` dùng để bỏ qua nhiễu/nhích nhẹ sau khi dừng xe. Nếu cần
điều tra sâu, có thể override evaluator bằng `--active-threshold`, nhưng không
được hạ ngưỡng chỉ để ép một bag nhiễu thành PASS.

Nếu quay CCW mà `gyro.z` âm, không sửa bằng covariance. Kiểm tra hướng gắn IMU,
`placement_axis_remap` và roll/pitch của transform.

Evaluator cố ý trả exit code khác 0 với bag đứng yên hoặc quay chưa đủ góc.
Không đổi report thành PASS bằng tay.

## 6. Lưu bằng chứng

Ghi số đo, sai số thước, hướng connector IMU và ảnh lắp đặt vào `report.md`.
Khi tất cả tiêu chí PASS, đổi duy nhất:

```yaml
status: measured_ccw_verified
```

trong `sensor_extrinsics.yaml`, build lại, rồi đóng session:

```bash
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Chỉ sau khi BNO055 có publisher và topic/freshness pass mới tuyên bố full
IMU/EKF fusion hoặc chạy gate cần gyro/yaw. Gate 2 longitudinal v2 chỉ dùng
`linear.x` filtered từ LiDAR/RF2O và vẫn phải ghi rõ BNO đang unavailable.
