# Kiến trúc canonical

## Luồng dữ liệu

```text
IMX708 → hardware_camera → /camera/front_camera/*
                           ↓
                    image_processing
                           ↓ /line/*
                 lane_reference_adapter
                           ↓ /lane/reference
A1M8 → /scan → /v2_rf2o_laser_odometry → conditioner ─┐
                                                      ├→ EKF → /odometry/filtered
BNO055 → /bno055/imu (gyro.z, tùy chọn khi healthy) ───┘
                                   │
A1M8 → /scan → rc_safe_corridor obstacle_detector → /perception/obstacles
/lane/reference + /perception/obstacles → safe_corridor_planner
                           ↓ /planning/corridor, /planning/behavior
               race_cars NMPC ←───┘
/scan + /control/nmpc + /mcu/arm → safety_monitor → /safety/estop, /actuation/arm=false
                           ↓ /control/nmpc
                    rc_actuation
                           ↓ /mcu/arm + /mcu/actuator_cmd
                    micro-ROS STM32
                           ↓ /mcu/status + /mcu/actuator_state
```

## Quy tắc sở hữu

- `image_processing` là package thị giác chính; không dùng `rc_perception`.
- `rc_safe_corridor` sở hữu nhận thức vật cản LiDAR (`/perception/*`),
  hành lang an toàn và FSM (`/planning/*`) và safety monitor (`/safety/*`).
  Safety monitor là node độc lập, chỉ đọc `/scan`, `/odometry/filtered`,
  `/control/nmpc`, `/mcu/arm` và được phép gọi `/actuation/arm false`.
- `race_cars` nhận `/planning/corridor` theo `corridor_mode`
  (`off`/`optional`/`required`); mặc định `optional` để thiếu corridor vẫn
  quay về NMPC bám làn như trước.
- `rc_interfaces` sở hữu message typed giữa perception, NMPC và actuation.
- `rc_localization` sở hữu topic conditioned và EKF. RF2O vẫn giữ package,
  executable và thuật toán vendor; chỉ tên node trong constructor runtime dùng
  `/v2_rf2o_laser_odometry` để tránh global `__node` remap làm sinh duplicate
  node nội bộ.
- `race_cars` mặc định chỉ phát dự đoán khi shadow; `/nmpc_cmd` vẫn là stop.
  Active NMPC chỉ chạy bằng override trong gate có record evidence.
- Chỉ `rc_actuation` được publish `/mcu/actuator_cmd` trong runtime được duyệt.
- `rc_bringup shadow_full.launch.py` không khởi động micro-ROS/actuation.
- `archive/` là read-only migration material, không tham gia build hay flash.

## Frame và topic

Body frame canonical là `base_link`. Sensor frames là `laser`, `imu_link`,
`camera_optical_frame`. Output localization chuẩn là `/odometry/filtered`;
không dùng alias cũ `/odom/filtered` hoặc `/odom_rf2o`.

Pose camera runtime hiện tại là `x=0.276 m`, `y=0 m`, `z=0.160 m` theo
`base_link`, với quaternion optical giữ nguyên; đây là cập nhật cơ khí mới và
chưa phải calibration cuối cùng. Pose LiDAR và IMU lấy từ
`rc_localization/config/sensor_extrinsics.yaml`. Nếu status còn là
**blocked_requires_measurement**, phải đo trước rolling test. Không được biến
giá trị provisional thành “đã calibration” chỉ vì TF tree kết nối.

## Nguồn cấu hình

[`config/vehicle_1_7.yaml`](../../config/vehicle_1_7.yaml) là index trỏ tới YAML
runtime của từng package. Tham số không được copy sang tài liệu; tài liệu chỉ
trích giá trị để giải thích. Mỗi thay đổi calibration phải có session evidence.

Trạng thái tổng thể và next action nằm ở [`project_state.md`](../../project_state.md).
README/step document không được giữ một snapshot độc lập mâu thuẫn với file đó.

## Profile launch

- `sensors_only.launch.py`: camera/lane + LiDAR/RF2O + IMU/EKF.
- `shadow_full.launch.py`: sensors + NMPC shadow, không actuator.
- `actuation_disarmed.launch.py`: agent + actuation, luôn khởi động DISARMED.
- `safe_corridor_full.launch.py`: sensors + `rc_safe_corridor` + NMPC
  (shadow mặc định, `nmpc_shadow:=false` chỉ trong gate có record);
  không actuator. Xem [`step06_safe_corridor.md`](step06_safe_corridor.md).

Không có profile “auto drive” mặc định. Trạng thái hiện tại là
`steering_calibrated=true`, `model_calibrated=true`, `shadow_mode=false` cho
hardware test có kiểm soát; active/rolling vẫn chỉ chạy sau pre-arm, held và
evidence an toàn tương ứng. Profile `shadow_full.launch.py` vẫn ép shadow.

Trong runtime ngày 2026-07-13, `/scan` và RF2O chạy khoảng 6,4 Hz; đây là
nguồn tần số của vận tốc dọc. `/bno055/imu` hiện chưa có publisher, nên không
được gọi là full IMU/EKF fusion cho tới khi driver BNO055 hoạt động lại.
