# Bước 2 — Fusion vận tốc LiDAR/RF2O + BNO055 + EKF

## Trạng thái runtime — 2026-07-16

Pipeline LiDAR/RF2O/EKF đang chạy và đã sửa lỗi duplicate node. Graph hiện tại
phải có đúng một node `/v2_rf2o_laser_odometry`, một `/sllidar_node`, một
`/rf2o_velocity_conditioner` và một `/ekf_filter_node`. `/odometry/rf2o_raw`
phải có đúng một publisher là `/v2_rf2o_laser_odometry`.

Quan sát hiện tại cho thấy `/scan` và `/odometry/rf2o_raw` khoảng `6,4 Hz`, còn
`/odometry/filtered` khoảng `6 Hz` khi không có nguồn IMU. Đây là hành vi phù
hợp với cấu hình đang lấy `vx` từ LiDAR; không được coi 6 Hz là lỗi chỉ vì EKF
parameter `frequency` là 50 Hz.

Evidence Gate 2 PASS ngày 2026-07-14 xác nhận `/scan` khoảng `6,50 Hz`, RF2O
raw khoảng `6,49 Hz` và `/odometry/filtered` khoảng `6,17 Hz` sau khi
`sensor_timeout` được đặt `0,15 s`. Gate 2 model-fit đã PASS với held-out RMSE
`0,007438 m/s`.

Tuy nhiên, trong Gate 2 v2 session
`evidence/20260713-171843_longitudinal_fit_rolling_v2_60s`, RF2O vẫn khoảng
`6,4 Hz` nhưng `/odometry/filtered` chỉ khoảng `3,65 Hz`, với các khoảng cách
giữa mẫu lên tới khoảng `0,32 s`. Đây là khác biệt runtime cần kiểm tra trước
khi chạy lại model-fit: `frequency: 50.0` không đảm bảo output thực tế 50 Hz,
nhưng filtered cũng không được để thưa đến mức làm fitter loại phần lớn
transition. Không kết luận nguyên nhân hoặc nới điều kiện fitter chỉ từ một
session; cần đo lại và đối chiếu timestamp/raw/filtered.

Nguyên nhân của cadence thấp đã được xác định: `ekf_hardware.yaml` đặt
`sensor_timeout: 0,3 s`; theo semantics của `robot_localization`, giá trị này
đặt cận dưới cho cadence output xấp xỉ `1/0,3 Hz`. Đã sửa tiếp thành `0,15 s`
để cho phép filtered vượt 6 Hz theo nguồn RF2O 6,4 Hz. Build và unit test
`rc_localization` `6/6` PASS; cần restart runtime thật để xác nhận cadence mới.

Gate 2 đã dùng filtered odometry cho probe `1595 us` và các lượt fit
`1580/1585/1590 us`. Không dùng việc tăng cadence để tạo mẫu giả; số lượng
transition trong evidence đến từ các mẫu odometry thực tế.

`/bno055/imu` hiện có **0 publisher** trong runtime. Do đó tài liệu này không
được dùng để tuyên bố BNO055/EKF fusion đầy đủ; Gate 2 dọc chỉ có thể ghi nhận
`vx` nếu `/odometry/filtered` còn fresh. Phải sửa BNO055 trước các gate cần
gyro.z/yaw hoặc đánh giá fusion IMU đầy đủ.

Gate 5 test 1–3 ngày 2026-07-16 vẫn chưa PASS. Test 3 dùng LiDAR/RF2O làm
nguồn odometry với `start_imu:=false`, filtered speed tối đa `0,504 m/s` và
safe-stop sau khi perception/control fault xuất hiện. Không thay đổi kết luận:
Gate 5 chưa PASS. Lượt Gate 5 cuối cũng chưa PASS; bag cuối có armed khoảng
`30,8 s` và
`solver_failed`/`lane_invalid`. Phiên kế tiếp là diagnostic servo-only, vẫn
dùng LiDAR/RF2O và không bật BNO055; xem prompt
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).
Nếu cần chẩn đoán tiếp, đối chiếu
timestamp `/scan`, RF2O và `/odometry/filtered` trong rosbag test 3.

## 1. Chuẩn bị

Package sử dụng:

- `sllidar_ros2`: publish `/scan`;
- `rf2o_laser_odometry` executable, runtime node
  `/v2_rf2o_laser_odometry`: publish `/odometry/rf2o_raw`;
- `rc_localization`: lọc outlier, smoothing và gán covariance;
- `bno055`: publish `/bno055/imu` khi driver healthy; hiện runtime chưa có
  publisher;
- `robot_localization`: publish `/odometry/filtered`.

EKF chỉ fuse:

- `vx` từ RF2O đã condition;
- `angular_velocity.z` từ BNO055.

Không fuse acceleration BNO055 trong phiên bản này.
Launch dùng `rc_localization/bno055_safe_node`: wrapper lifecycle mỏng quanh
driver BNO055 đã ghim, nhằm tránh lỗi upstream shutdown `rclpy` hai lần khi
Ctrl+C; thuật toán đọc sensor không bị thay thế.

Baseline đứng yên lịch sử ngày 2026-06-29:

- BNO055 gần 100 Hz nhưng `ax` vẫn có bias trung bình `0,01108 m/s²`; tích phân
  ngây thơ 60 giây có thể tạo drift khoảng `0,66 m/s`;
- RF2O raw ở 6,56 Hz có stddev chỉ `0,000339 m/s`;
- conditioner và EKF giữ `vx=0`, diagnostics toàn level 0.

Các số baseline trên không thay thế kiểm tra runtime hiện tại; đặc biệt không
được ghi BNO055 là đang publish nếu `ros2 topic info -v /bno055/imu` báo
publisher count bằng 0.

Vì vậy nguyên nhân cũ không phải “IMU nhanh hơn LiDAR”, mà là fuse/tích phân
acceleration có bias với covariance/frame chưa đúng. V2 dùng gyro.z của IMU,
không dùng acceleration để tạo vận tốc.

Build:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/bootstrap.sh
bash scripts/build.sh rc_localization rc_validation
bash scripts/test.sh rc_localization rc_validation
```

Kiểm tra thiết bị:

```bash
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null
```

Hai static TF hiện để `0,0,0` vì archive cũ không có extrinsic đã đo. Trước
rolling test phải đo pose LiDAR/IMU và xác nhận quay xe CCW cho `gyro.z > 0`.
Launch đã expose `lidar_{x,y,z,roll,pitch,yaw}` và
`imu_{x,y,z,roll,pitch,yaw}`; không cần sửa source để nhập số đo. Đơn vị là mét
và radian, theo transform `base_link → sensor`.

## 2. Lệnh chạy

Terminal 1:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/run_fusion.sh
```

Nếu tên cổng khác:

```bash
bash scripts/run_fusion.sh \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  imu_port:=/dev/ttyAMA0
```

Không dùng `sudo` để chạy node. User phải thuộc group `dialout`.

## 3. Quan sát

Terminal 2:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/observe_fusion.sh
```

Quan sát riêng:

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/rf2o_raw
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --field twist.twist.linear.x
ros2 topic echo /diagnostics
ros2 topic info -v /odometry/rf2o_raw
ros2 topic info -v /bno055/imu
```

Không đánh giá bằng việc nhìn một con số. Cần ghi đủ 60 giây và tính thống kê.

## 4. Xuất dữ liệu

Tạo session đứng yên:

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario fusion_stationary)
bash scripts/evidence/record.sh --session "$SESSION" --duration 60
bash scripts/evidence/evaluate.sh --session "$SESSION" \
  --mode fusion --bag "$SESSION/rosbag/latest" --expected-speed 0.0
```

`rosbag/latest` trỏ tới thư mục mà `record.sh` vừa tạo. Cách này đánh giá đúng
dữ liệu trong bag, không yêu cầu xe lặp lại cùng kịch bản. Chế độ subscribe
live vẫn có thể dùng để theo dõi nhanh:

```bash
bash scripts/evidence/evaluate.sh --session "$SESSION" \
  --mode fusion --duration 60 --expected-speed 0.0
```

Chạy đều mục tiêu `0,2 m/s` (chỉ sau khi có BNO/runtime fusion phù hợp):

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario fusion_constant_0p2)
bash scripts/evidence/record.sh --session "$SESSION" --duration 60
bash scripts/evidence/evaluate.sh --session "$SESSION" \
  --mode fusion --bag "$SESSION/rosbag/latest" --expected-speed 0.2
```

`record.sh` đã ghi rosbag bằng process group riêng và chỉ dừng recorder do nó
tạo. Có thể truyền danh sách topic sau dấu `--` nếu cần thu hẹp dữ liệu.

## 5. Đánh giá

Evaluator tạo:

- `velocity_samples.csv`;
- `metrics.json`;
- `report.md`;
- metadata môi trường/thiết bị;
- `SHA256SUMS`.

Gate đứng yên:

- `|mean(vx)| ≤ 0,02 m/s`;
- P95 absolute error ≤ `0,05 m/s`;
- không spike quá `0,5 m/s`.

Gate chạy đều:

- standard deviation ≤ `0,03 m/s`;
- mean error ≤ `max(0,03 m/s, 15%)`;
- không spike quá `0,5 m/s`.

Nếu raw nhiễu nhưng conditioned/EKF pass thì pipeline hợp lệ. Nếu cả conditioned
và EKF fail, giữ NMPC ở shadow mode và tune bằng rosbag; không cấp motor tự hành.

## 6. Lưu bằng chứng

Evidence nằm tại:

```text
evidence/<timestamp>_<scenario>/
```

Khi báo cáo, mở `report.md`, `csv/fusion/metrics.json` và CSV. Đóng session:

```bash
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Không ghi PASS nếu chưa đạt số liệu. Nếu đổi parameter, tạo session mới.
