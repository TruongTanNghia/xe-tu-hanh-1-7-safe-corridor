# Autonomous Car Hardware Platform V2

ROS 2 Humble native cho xe Ackermann 1/7 trên Raspberry Pi 5, Debian 12 arm64.

## Đọc trước khi làm việc

Nguồn trạng thái duy nhất của project là [`project_state.md`](project_state.md).
File này ghi rõ code/interface đang dùng, evidence mới nhất, blocker và next
action. README này chỉ là cửa vào nhanh; nếu README và project state có vẻ khác
nhau, dùng `project_state.md` và cập nhật README trong cùng một thay đổi.

Mục lục vận hành tiếng Việt: [`docs/vi/README.md`](docs/vi/README.md).

## Trạng thái tóm tắt — 2026-07-16

- **2026-10-22:** thêm stack Safe Corridor (`rc_safe_corridor`: vật cản LiDAR
  → hành lang Frenet → FSM → NMPC ràng buộc hành lang + safety monitor).
  Code + test offline xong, chưa có runtime evidence; xem
  [`docs/vi/step06_safe_corridor.md`](docs/vi/step06_safe_corridor.md).
- Camera + `image_processing` đã có bản sửa trong working tree; output camera là
  `640x360 bgr8`, lane ROI mặc định hiện là `40–63%` chiều cao ảnh và scan
  ngang `0–100%` sau khi camera được đưa ra phía trước xe.
- Evidence tĩnh mới nhất gồm ba pose, đều `line_detected` với confidence `1.0`:
  [`evidence/lane_pose_20260711_154156`](evidence/lane_pose_20260711_154156).
- Full-loop shadow + micro-ROS/actuation DISARMED đã PASS:
  [`evidence/20260711-172406_full_loop_actuation_disarmed`](evidence/20260711-172406_full_loop_actuation_disarmed).
- Runtime actuation hiện có `steering_calibrated=true`,
  `esc_calibrated=true`; NMPC hardware config mặc định dùng
  `shadow_mode=false` cho các phiên test có kiểm soát và đã merge model
  candidate với `model_calibrated=true`.
- Gate 2 wheels-up cũ FAIL vì LiDAR tắt nên longitudinal recorder thu 0 mẫu;
  session lịch sử này đã được dọn khỏi Pi và không dùng để fit.
- Gate 2 v2 model-fit đã PASS tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
  184 armed transitions, đủ PWM `1580/1585/1590 us`, held-out RMSE
  `0,007438 m/s`, bag `59,4s`.
- Actuation/control runtime đã nâng lên khoảng `50 Hz`; filtered odometry trong
  evidence khoảng `6,17 Hz`; guard Gate 2 là `0,6 m/s`.
- Runtime RF2O hiện là `/v2_rf2o_laser_odometry`, `/odometry/rf2o_raw` có đúng
  một publisher. `/scan`/RF2O và filtered hiện khoảng 6 Hz khi LiDAR là nguồn
  vận tốc; `/bno055/imu` hiện chưa có publisher runtime.
- Candidate model dọc đã được review/merge; Gate 3 active pre-arm PASS tại
  `evidence/20260715-095303_gate3_default_model_prearm_20260715`. Held active
  đã PASS theo trạng thái dự án; rolling slow Gate 5 vẫn cần evidence riêng.
- Gate 5 đã có ba lượt thử full map trong ngày 2026-07-16; cả ba chưa PASS.
  Lượt 3 dùng exposure `4000 us`, active khoảng `21,5 s`, tốc độ filtered tối
  đa `0,504 m/s`, sau đó safe-stop do command/lane invalid. Xem
  [`test 3 report`](evidence/20260716-115602_gate5_full_map_rolling_exposure4000_20260716/report.md)
  và [`evidence/test3_gate5.mp4`](evidence/test3_gate5.mp4).
- So với exposure `8000 us`, `4000 us` cho thời gian active dài hơn và lane ổn
  định hơn ở đoạn đầu, nhưng chưa chứng minh được độ bền khi vào cua gắt.
  Hướng điều tra hiện tại là tăng robustness của image processing theo ánh
  sáng và hình học cua; chỉ khi perception ổn định mà lỗi còn lặp lại mới
  quy kết sang NMPC hoặc tầng khác.
- Quy trình lệnh thủ công đầy đủ cho lần test kế tiếp nằm tại
  [`docs/vi/gate5_manual_test.md`](docs/vi/gate5_manual_test.md).
- Lượt camera-capture mới nhất bị BLOCKED/INCOMPLETE trước khi xe chạy: bag
  [`20260716-123801_gate5_camera_capture_exposure4000_20260716`](evidence/20260716-123801_gate5_camera_capture_exposure4000_20260716)
  có ảnh raw/debug/fitting liên tục nhưng arm tự safe-stop; không tính là
  Gate 5 PASS.
- Working tree hiện không sạch; không coi các kết quả lịch sử là verification
  của mọi thay đổi cục bộ hiện tại.
- Lượt Gate 5 cuối ngày 2026-07-16 tại
  `evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716` armed
  khoảng `30,8 s`, phát sinh `lane_invalid`/`solver_failed` và safe-stop, nên
  Gate 5 vẫn **chưa PASS**.
- Bước tiếp theo là diagnostic image processing + servo-only: vô hiệu hóa
  motor/ESC vật lý, đẩy xe bằng tay qua cua, ghi raw/debug/fitting image cùng
  `cte`, `kappa`, `s`, `heading` và servo PWM. Xem
  [`prompt_gate5_image_diagnostic.md`](docs/vi/ai_workflow/prompt_gate5_image_diagnostic.md).
  Runtime hiện chưa có `servo_only`; không dùng `esc_calibrated=false` như
  workaround vì nó chặn arm.
- Image processing đã thêm `boundary_corridor` và phục hồi `partial-boundary`
  ngày 2026-07-17: fit riêng các đoạn biên còn nhìn thấy; khi một biên mất tạm
  thời, dùng độ rộng corridor của frame hợp lệ trước đó tối đa `150 ms`; vùng
  tìm biên mặc định bao phủ toàn chiều rộng ảnh để không mất biên sát mép khi
  vào cua. Build package PASS; cần test runtime lại trước khi coi là đã giải
  quyết Gate 5.

## Bootstrap, build và test

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/bootstrap.sh
bash scripts/build.sh
bash scripts/test.sh
```

Nếu chỉ làm camera/lane:

```bash
bash scripts/build.sh hardware_camera image_processing
bash scripts/run_lane_detection.sh
bash scripts/observe_lane_web.sh       # VS Code forward port 8080
```

Mỗi terminal runtime cần source ROS underlay và workspace:

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
```

Dependency được ghim tại `dependencies.repos`. `ros2_ws/src/vendor` là output
của `vcs import`, không sửa trực tiếp. Nếu cần acados hoặc rosbag2, dùng các
script bootstrap tương ứng trong `scripts/`.

## Runtime profiles

```bash
ros2 launch rc_bringup sensors_only.launch.py
ros2 launch rc_bringup shadow_full.launch.py
```

`shadow_full.launch.py` không khởi động micro-ROS/actuation và ép
`shadow_mode=true`. Không có profile auto-drive mặc định. Active NMPC chỉ chạy
sau khi
pre-arm gate sạch và đang record evidence.

## Safety

- Không dừng process do session khác tạo.
- Không cấp lệnh motor trước khi pass các gate tương ứng. Gate 2 v2 là test
  rolling có kiểm soát trên sàn, không dùng wheels-up calibration source.
- Chỉ `rc_actuation` được phép publish `/mcu/actuator_cmd`.
- Không flash firmware trong `archive/`; dùng source dưới `firmware/` và chạy
  `bash scripts/verify_firmware_source.sh`.
- `steering_calibrated=true` hiện đã được bật theo xác nhận wheels-up/steering
  gate. `model_calibrated=true` đã được bật sau review candidate và Gate 3
  pre-arm; `shadow_mode=false` là mặc định cho hardware test. Profile
  `shadow_full.launch.py` vẫn ép `shadow_mode=true`.

Xem [`project_state.md`](project_state.md) để biết việc tiếp theo thay vì suy
đoán từ các mốc lịch sử trong tài liệu cũ.
