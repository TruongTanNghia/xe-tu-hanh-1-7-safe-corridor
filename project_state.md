# Project state — Autonomous Car Hardware Platform V2

> Đây là nguồn trạng thái canonical của repo. Cập nhật file này sau mỗi thay đổi
> làm thay đổi cách build, runtime, interface, calibration hoặc gate kiểm thử.
> README chỉ là cửa vào; tài liệu chi tiết phải trỏ về trạng thái ở đây.

**Snapshot:** 2026-07-16
**Workspace:** `/home/ac/main/1_projects/autonomous_car_hardware_platform_v2`  
**Platform:** Raspberry Pi 5 8 GB, Debian 12 arm64, ROS 2 Humble native  
**Vehicle:** Ackermann 1/7, `vehicle_id=autonomous_car_1_7`

### Camera extrinsic cập nhật — 2026-07-27

Static TF `base_link → camera_optical_frame` trong
`hardware_camera/launch/lane_detection.launch.py` đã cập nhật theo số đo cơ
khí mới: `x=0.276 m`, `y=0 m`, `z=0.160 m`; quaternion hướng camera giữ
nguyên. Thay đổi tương ứng là tăng `x` thêm `13 cm` và giảm `z` `4,5 cm` so
với cấu hình trước. Đây là pose provisional cần kiểm tra lại bằng live image,
scanband và CTE; chưa coi là calibration cuối cùng.

### Safe Corridor (vật cản + hành lang an toàn) — 2026-10-22

Đã thêm package `rc_safe_corridor` (obstacle_detector, safe_corridor_planner,
safety_monitor), message `Obstacle/ObstacleArray/Corridor/BehaviorState`,
mở rộng `LaneReference` (`evaluation_forward_m`, `path_lateral_m`) và sửa NMPC
nhận `/planning/corridor` (ràng buộc mềm theo stage + footprint 2 vòng tròn,
`corridor_mode: optional`). Launch: `rc_bringup safe_corridor_full.launch.py`;
script: `scripts/run_safe_corridor.sh`, `scripts/corridor_offline_demo.sh`.
Unit test offline PASS (44 test `rc_safe_corridor`, 5 test `corridor_lookup`);
**chưa build trên Pi với acados và chưa có runtime evidence**. Thiết kế và
mốc nghiệm thu GĐ0–GĐ5 ở [`docs/vi/step06_safe_corridor.md`](docs/vi/step06_safe_corridor.md).
Next action: build `rc_interfaces rc_safe_corridor race_cars rc_bringup` trên
Pi, chạy `run_safe_corridor.sh full` (shadow) với vật cản cao > 0,22 m, xác
nhận `/perception/obstacle_status`, `/planning/status`, RViz corridor, rồi
test safety monitor 10/10 trước khi active NMPC ở 0,3 m/s.

## 1. Kết luận hiện tại

> Phần dưới đây là snapshot mới nhất và có quyền ưu tiên hơn các mốc lịch sử
> cũ hơn trong file này.

### Gate 5 — rolling slow full map, 2026-07-16

Ba lượt thử Gate 5 gần nhất đều **chưa PASS**:

- test 1: `evidence/20260716-113755_gate5_rolling_slow_20260716`;
- test 2: `evidence/20260716-114946_gate5_full_map_rolling_20260716`;
- test 3: `evidence/20260716-115602_gate5_full_map_rolling_exposure4000_20260716`.

Test 3 dùng manual exposure `4000 us`, analogue gain `1.0`, active khoảng
`21,5 s`, tốc độ `/odometry/filtered` tối đa `0,504 m/s`, không reverse, và
actuation kết thúc ở safe output `servo=1500`, `esc=1550`. Rosbag ghi nhận
`ACTIVE_VALID=1100`, `lane_invalid=18`, `cte_outside_safe_bound=7` và
`solver_failed=2`; vì vậy hệ thống đã safe-stop trước khi đạt ba vòng liên
tiếp. Video quan sát là [`evidence/test3_gate5.mp4`](evidence/test3_gate5.mp4).

Video cho thấy xe vượt đoạn chuyển làn đơn rồi đi vào cua phải gắt, có thời
điểm chạy sát biên ngoài; video một mình không đủ chứng minh chính xác trigger
safe-stop. Kết luận tạm thời là cần ưu tiên kiểm tra độ ổn định của image
processing khi ánh sáng thay đổi và khi vào cua. Nếu perception đã ổn định mà
safe-stop vẫn lặp lại, tiếp tục phân tích NMPC, reference, odometry và các
tầng còn lại. Không ghi nhận Gate 5 PASS khi chưa đủ ba vòng và fault-free.

Quy trình chạy lại thủ công được chuẩn hóa tại
[`docs/vi/gate5_manual_test.md`](docs/vi/gate5_manual_test.md).

Lượt kiểm tra tiếp theo ngày 2026-07-16
`evidence/20260716-123801_gate5_camera_capture_exposure4000_20260716` đã ghi
được rosbag `59,3 s` với raw/debug/fitting image liên tục, nhưng **INCOMPLETE**:
arm service trả `ACTIVE` rồi actuation lập tức safe-stop về `DISARMED`,
`/mcu/arm` trong bag đều `False`, xe không chạy. Fusion đứng yên PASS; chưa có
active lap và chưa được tính là Gate 5 evidence PASS. Cần phân tích trigger
safe-stop/solver và các frame ảnh tương ứng trước khi chạy lại.

Lượt cuối ngày 2026-07-16 tại
`evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716` chạy lại
full map với manual exposure `4000 us`, analogue gain `1.0`, chỉ ghi rosbag và
CSV, không ghi ảnh. Bag dài khoảng `138,1 s`, trong đó `/mcu/arm=true` khoảng
`30,8 s` (`19,134–49,934 s` tính từ đầu bag), chưa đủ ba vòng liên tiếp. Trong
khoảng armed có `570 ACTIVE_VALID`, `21 lane_invalid`, `16 solver_failed:4`,
`5 cte_outside_safe_bound` và `4 line_low_confidence`; terminal NMPC ghi nhiều
lỗi `ACADOS_MINSTEP`. Actuation đã safe-stop về `DISARMED`, PWM `1500/1550`.
Kết luận lượt cuối: **Gate 5 FAIL / INCOMPLETE**, không được ghi nhận PASS.
Không có image topics trong session này; cần dùng các session camera trước đó
cho phân tích perception.

Kế hoạch phiên kế tiếp không phải autonomous Gate 5 ngay lập tức. Operator đã
chọn diagnostic image processing + servo-only: vô hiệu hóa motor/ESC vật lý,
đẩy xe bằng tay qua đường thẳng và cua, ghi raw/debug/fitting image cùng
`cte`, `kappa`, `s`, `heading`, lane reference và servo PWM. Runtime
`actuation_manager` hiện chưa có `servo_only`; không dùng `esc_calibrated=false`
để giả lập vì tham số đó chặn arm. ESC phải giữ `1550` trong toàn phiên. Chỉ
sau khi đối chiếu perception và servo theo cùng timestamp mới quyết định sửa
image processing hay chuyển điều tra NMPC. Prompt chat mới là
`docs/vi/ai_workflow/prompt_gate5_image_diagnostic.md`.

Đã dọn evidence dư thừa sau phiên 2026-07-16: phân vùng root hiện còn khoảng
`18 GB` khả dụng (`58 GB` tổng, dùng khoảng `69%`), thư mục `evidence` còn
khoảng `3,2 GB`. Giữ lại một evidence PASS canonical cho các gate đã đạt:
Gate 2 model-fit, Gate 3 default-model pre-arm, Gate 4 held active; đồng thời
giữ full-loop DISARMED PASS, lane-pose baseline, Gate 2 diagnostic được tài
liệu tham chiếu, video test 3, session camera có ảnh và session Gate 5 cuối.
Các rosbag/image trùng hoặc không còn cần cho quyết định hiện tại đã xóa; Gate
5 chưa có evidence PASS nên không tạo hoặc giữ một bản PASS giả định.

Gate 4 held active đã được operator xác nhận PASS tại
`evidence/20260716-104744_gate4_held_active_manual_exposure_20260716`; các
session fault/reconnect cùng ngày cũng được lưu riêng. Hardware IWDG vẫn tắt
trong firmware hiện tại, nên không ghi nhận independent watchdog là điều kiện
PASS.

Pipeline camera + lane detection đã được sửa trong working tree. Camera publish
ROS image encoding `bgr8`, ROI lane mặc định hiện là `40–63%` sau khi camera được
đưa ra phía trước xe; scan ngang mặc định đã mở toàn ảnh `0–100%` để quan sát
hai mép biên. Track thực tế là đường
đen với hai biên vàng. Evidence tĩnh mới nhất
[`evidence/lane_pose_20260711_154156`](evidence/lane_pose_20260711_154156)
có ba pose `straight`, `enter_curve`, `exit_curve`; cả ba đều
`line_detected`, confidence `1.0`.

Full-loop shadow có micro-ROS/actuation DISARMED đã PASS tại
[`evidence/20260711-172406_full_loop_actuation_disarmed`](evidence/20260711-172406_full_loop_actuation_disarmed):

- bag 59,3 s, 27.925 messages;
- `/line/status=line_detected` toàn bộ 1.682 mẫu;
- fusion evaluator PASS trong session lịch sử; runtime LiDAR hiện tại phát
  `/scan` và RF2O khoảng 6,4 Hz, nên `/odometry/filtered` cũng quan sát khoảng
  6 Hz khi không có nguồn IMU. Đây là tần số nguồn đo đang dùng cho `vx`, không
  phải lỗi cần ép lên 50 Hz;
- LiDAR snapshot PASS;
- `/nmpc_cmd` luôn `[0,0]`, `/control/nmpc.valid` không bao giờ true;
- `/actuation/status` luôn `state=DISARMED`;
- `/mcu/arm` không bao giờ true;
- `/mcu/actuator_cmd` luôn `(1500,1550)`;
- `/mcu/actuator_state` luôn `[1500,1550,29]`;
- publisher duy nhất của `/mcu/actuator_cmd` là `actuation_manager`.

Trạng thái tổng thể hiện tại:

- `shadow_full.launch.py` vẫn không launch micro-ROS/actuation;
- actuation chạy bằng `scripts/run_micro_ros_actuation.sh` và luôn khởi động
  DISARMED;
- `ros2_ws/src/rc_actuation/config/actuation_hardware.yaml` hiện có
  `steering_calibrated=true`, `esc_calibrated=true`;
- `ros2_ws/src/race_cars/config/nmpc_hardware.yaml` hiện có
  `shadow_mode=false`, `model_calibrated=true`; model candidate đã được review
  và merge có chủ đích cho các phiên hardware test có kiểm soát. Profile
  `shadow_full.launch.py` vẫn ép shadow;
- Gate 5 rolling slow chưa được duyệt vì ba lượt full-map gần nhất chưa đủ ba
  vòng liên tiếp và còn fault perception/control; Gate 2 model-fit hiện đã
  PASS.
- Gate 2 wheels-up cũ đã FAIL vì LiDAR tắt nên recorder thu 0 mẫu; session
  lịch sử này đã được dọn khỏi Pi. Không dùng nó để fit.
- Đã thêm Gate 2 v2: rolling calibration có kiểm soát trên sàn, LiDAR bật,
  source riêng
  `scripts/run_esc_rolling_calibration.sh` và không chạy NMPC đồng thời.
- Probe PWM `1595 us` đã hoàn tất trong
  `evidence/20260714-095327_longitudinal_probe_1595_hold6s`: rosbag khoảng
  `19,3 s`, CSV 123 mẫu, vận tốc plateau khoảng `0,516 m/s`, đỉnh
  `0,538 m/s`. Dựa trên kết quả này, operator đã chốt
  `max_speed_mps=0,6 m/s` cho các lượt fit Gate 2; guard `1,0 m/s` chỉ là
  guard tạm thời của probe cũ.
- Rolling source dùng odometry freshness timeout mặc định `0,6 s`; timeout
  `0,3 s` trước đó quá sát với jitter của LiDAR/RF2O khoảng 6,4 Hz và đã gây
  abort ở `0,304 s`.
- Đã sửa duplicate node RF2O: node chính là `/v2_rf2o_laser_odometry`, không
  còn hai node cùng tên. `/odometry/rf2o_raw` hiện có đúng một publisher.
- Runtime hiện tại `/bno055/imu` có 0 publisher dù EKF vẫn khai báo subscribe
  gyro.z. Vì vậy không được tuyên bố BNO/EKF fusion đầy đủ cho tới khi BNO055
  healthy; Gate 2 dọc vẫn có thể dùng `vx` từ LiDAR/RF2O nếu freshness đạt.
- Gate 2 v2 model-fit đã PASS trong
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
  rosbag `59,4 s`, `184` armed transitions, đủ PWM `1580/1585/1590 us`,
  actuation/control khoảng `50 Hz`, filtered khoảng `6,17 Hz`, armed peak
  `0,381 m/s` dưới guard `0,6 m/s`, train RMSE `0,021743 m/s` và held-out
  RMSE `0,007438 m/s`. Gate 2 model-fit được chốt PASS.
- Review phát hiện process snapshot của session trên thực tế dùng
  `stop_duration_s:=1.0`, dù scenario đặt tên `stop2p5`; vì vậy evidence này
  không chứng minh riêng hành vi dừng `2,5 s`. Candidate có `model.cr3=20`
  chạm bound trên của fitter. Review mới xác nhận RMSE vẫn đạt gate, nhưng
`cr3` chưa được nhận dạng mạnh; giữ đây là rủi ro cần theo dõi khi chạy
  rolling, không tự ý retune chỉ để đạt gate.
- Gate 3 default-model active pre-arm đã PASS tại
  `evidence/20260715-095303_gate3_default_model_prearm_20260715`: bag
  `148,78 s`, fusion/LiDAR PASS, model runtime đọc `cr3=20.0`, toàn bộ MCU
  DISARMED và `/mcu/arm=false`. Có 10 lần `lane_invalid` được safe-stop đúng
  thiết kế; không gọi arm.
- Gate 2 v2 rolling session gần nhất
  `evidence/20260713-171843_longitudinal_fit_rolling_v2_60s` đã ghi được rosbag
  59,48 s và longitudinal CSV 221 mẫu. CSV có 37 mẫu `armed=1`, trong đó 6
  mẫu ở PWM 1580; vì vậy arm/MCU armed flag đã hoạt động. Tuy nhiên sequence
  bị abort khi `vx` đạt khoảng `0,400 m/s`, chỉ tạo được 15 transition hợp lệ
  (yêu cầu tối thiểu 50), chưa đi qua các mức PWM còn lại và chưa có fit report.
- Trong session rolling lịch sử, `/scan` và RF2O khoảng 6,4 Hz nhưng
  `/odometry/filtered` chỉ khoảng 3,65 Hz; fitter hiện loại transition có
  `dt > 0,25 s`. Đây là vấn đề cần xử lý trước khi chạy lại Gate 2, cùng với
  việc xác minh vận tốc `0,400 m/s`. Chưa dùng `TEMP_GUARD=1,0 m/s` làm guard
  chính thức; ghi chú “chưa bật `model_calibrated=true`” chỉ áp dụng cho
  session lịch sử này, không phản ánh config runtime hiện tại.
- Phân tích rosbag cho thấy tại step PWM 1580, RF2O raw đạt đỉnh khoảng
  `0,474 m/s`; tốc độ tính độc lập từ delta pose cũng khoảng `0,46–0,50 m/s`,
  rồi conditioner khoảng `0,403 m/s` và filtered khoảng `0,443 m/s`. Vì vậy
  cảnh báo tốc độ không phải chỉ do EKF phóng đại. Guard `0,35 m/s` đang dừng
  đúng theo guard cũ; kết quả probe mới đã được dùng để chốt guard Gate 2 ở
  `0,6 m/s`.
- Gate 2 model-fit đã dùng ba mức PWM `1580/1585/1590 us`, với
  `max_speed_mps=0,6 m/s`. PWM `1595 us` chỉ dùng cho probe xác định guard,
  không đưa vào fit.
- Vì đường test ngắn, các mức PWM sẽ được chạy thành nhiều lượt ngắn có stop,
  disarm và đưa xe về vị trí xuất phát; dữ liệu các lượt phải được aggregate
  trong cùng evidence trước khi fit. Không yêu cầu xe chạy thẳng liên tục 60 s.
- Đã sửa `rc_localization/config/ekf_hardware.yaml`: `sensor_timeout` từ `0,3`
  xuống `0,15 s` để cho phép filtered vượt 6 Hz, phù hợp RF2O khoảng 6,4 Hz.
  Runtime Gate 2 mới xác nhận filtered khoảng `6,17 Hz`; build và unit test
  `rc_localization` `6/6` PASS.

Working tree không sạch. Không coi `HEAD` là toàn bộ source/config đã chạy trong
các session gần nhất.

## 2. Runtime hiện tại

Camera/lane pipeline:

```text
IMX708 → /camera/front_camera/image_raw (bgr8)
       → image_processing
       → /line/status, /line/confidence, /line/cte,
         /line/heading_angle, /line/kappa, /line/reference_path,
         /line/debug_image, /line/fitting_image
       → lane_reference_adapter
       → /lane/reference
```

Ngày 2026-07-17 đã thêm bản sửa `boundary_corridor` và phục hồi
`partial-boundary` trong `image_processing`: các đoạn biên còn nhìn thấy được
fit riêng; khi một biên mất tạm thời, dùng độ rộng corridor của frame hợp lệ
trước đó tối đa `150 ms`. Parameter runtime là
`boundary_corridor_enabled=true`, `boundary_margin_px=4`,
`boundary_recovery_enabled=true`, `boundary_recovery_max_age_ms=150` và
`boundary_recovery_min_observations=3`. Vùng tìm biên đã mở rộng mặc định toàn
chiều rộng ảnh bằng `boundary_search_left_ratio=0.0` và
`boundary_search_right_ratio=1.0`, vì biên có thể nằm sát mép ảnh khi vào cua.
Nếu không đủ dữ liệu, pipeline không lấy mép ảnh làm biên giả mà fallback/safe-stop. Build package
`image_processing` và `hardware_camera` PASS, package test không có lỗi, nhưng
chưa có runtime evidence mới nên chưa kết luận fix đã PASS trên xe. Sau ảnh
`image copy 7.png` và `image copy 8.png`, đã chặn việc clamp polynomial vượt
ngoài ảnh vào mép ảnh; hàng corridor không hợp lệ được bỏ qua để tránh lấy
pixel đen ngoài lane.

Web debug đã được mở rộng thành hai trang trong cùng một lệnh
`bash scripts/observe_lane_web.sh`: Segmentation (raw/ROI/yellow/Otsu/corridor/
final scan mask) và Tracking / Output (boundary geometry/scan bands/poly
fitting/metrics). Các stage image nằm dưới `/line/debug/*`, mode nằm ở
`/line/debug/mode`. Build `hardware_camera` và `image_processing` PASS; chưa
coi viewer/stage mask là runtime evidence Gate 5.

Ngày 2026-07-27 đã triển khai bước 1 ổn định boundary: robust fit với MAD/sigma
clipping cho observation trái/phải. Build `image_processing` PASS, chưa có
runtime evidence sau fix; cần operator test lại cùng vị trí trước khi thêm
temporal smoothing.

Localization:

```text
/scan → /v2_rf2o_laser_odometry → /odometry/rf2o_raw
      → velocity_conditioner → /odometry/rf2o
      → EKF → /odometry/filtered

/bno055/imu → EKF (gyro.z cho yaw-rate; hiện chưa có publisher runtime).
Tốc độ dọc NMPC và longitudinal recorder đọc từ
`/odometry/filtered.twist.twist.linear.x`.
```

Hardware test mặc định dùng active NMPC; profile shadow vẫn có sẵn:

```text
/nmpc_cmd_shadow = dự đoán [steering, drive_effort]
/control/nmpc.valid = false
/control/nmpc.reason = SHADOW_MODE khi input hợp lệ
/nmpc_cmd = [0,0]
```

Actuation:

```text
actuation_hardware.yaml:
  steering_calibrated: true
  esc_calibrated: true
  control_rate_hz: 50.0
  servo_center_us: 1500
  esc_stop_us: 1550
  esc_forward_start_us: 1580
  esc_max_us: 1600

nmpc_hardware.yaml:
  shadow_mode: false
  model_calibrated: true
  model.cr3: 20.0
```

## 3. Evidence và mốc kiểm chứng

Các kết quả có giá trị hiện tại:

- 2026-06-29: full build 17/17 package PASS;
- 2026-06-29: acceptance test 25/25 cho tám package do V2 sở hữu;
- 2026-06-29: A1M8 live web/SSH và rosbag replay PASS;
- 2026-06-29: fusion đứng yên 59,48 s PASS, EKF 50 Hz, không spike;
- 2026-06-29: IMU CCW retry PASS, `gyro.z` đúng dấu khi quay CCW;
- 2026-06-29: NMPC shadow software benchmark 500 solve, 0 failure,
  P99 `0,387 ms`;
- 2026-07-11: lane evidence tĩnh ba pose PASS quan sát;
- 2026-07-11: full-loop shadow + micro-ROS/actuation DISARMED PASS;
- 2026-07-11: arm rejection khi `steering_calibrated=false` PASS trước khi
  steering gate được bật;
- 2026-07-11: `steering_calibrated=true` được bật trong config actuation theo
  xác nhận vận hành wheels-up/steering gate.
- 2026-07-13: phát hiện và sửa duplicate RF2O node do global `__node` remap;
  runtime xác nhận một `/v2_rf2o_laser_odometry` và một publisher raw.
- 2026-07-13: Gate 2 wheels-up session FAIL, longitudinal CSV có 0 mẫu vì
  LiDAR đã tắt; session lịch sử đã được dọn khỏi Pi.
- 2026-07-13: Gate 2 v2 rolling session
  `20260713-171843_longitudinal_fit_rolling_v2_60s` có rosbag/CSV hợp lệ và
  xác nhận armed flag hoạt động, nhưng chưa đạt Gate 2: sequence abort ở
  `0,400 m/s`, chỉ có 15/50 transition hợp lệ, chưa đủ đa dạng PWM và chưa có
  model-fit report. Session này được giữ làm evidence chẩn đoán, không dùng để
  bật model calibrated.
- 2026-07-13: Phân tích bag xác nhận tốc độ cao tại PWM 1580 có trong RF2O raw
  và delta pose; không thay đổi speed limit. Sửa EKF `sensor_timeout=0,15 s`,
  build `rc_localization` PASS và unit test `6/6` PASS. Runtime mới cần được
  restart để kiểm chứng filtered trên 6 Hz.
- 2026-07-14: Gate 2 v2 model-fit PASS tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`;
  held-out RMSE `0,007438 m/s`, đủ 184 armed transitions và ba mức PWM.
  Runtime actuation/control xác nhận khoảng 50 Hz; `rc_actuation` build/test
  `7/7` PASS.
- 2026-07-15: candidate được sinh lại với `model.cr3: 20.0`, default config và
  `DEFAULT_MODEL_PARAMETERS` được cập nhật; unit test `6/6`, build hai package
  PASS. Gate 3 active pre-arm với override `shadow_mode:=false` đã PASS; sau
  đó hardware config được chuyển sang `shadow_mode=false` cho các phiên test
  có kiểm soát;
  xem `evidence/20260715-095303_gate3_default_model_prearm_20260715`.

Các kết quả lịch sử chưa thay thế gate còn lại:

- `evidence/20260629-204550_powered_1580_12s` là FAIL cũ trước khi sửa đường
  velocity conditioner;
- các session cũ chưa có report model dọc dạng
  `plots/longitudinal_model/nmpc_model_candidate.yaml`;
- active/rolling session cũ tồn tại nhưng không được coi là gate hiện tại nếu
  thiếu report PASS rõ ràng.

## 4. Lệnh runtime canonical

Mỗi terminal runtime:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
```

NMPC/acados:

```bash
source scripts/acados_env.sh
```

Sensors không kèm NMPC:

```bash
ros2 launch rc_bringup shadow_full.launch.py start_nmpc:=false
```

Shadow full mặc định:

```bash
ros2 launch rc_bringup shadow_full.launch.py
```

micro-ROS + actuation manager:

```bash
bash scripts/run_micro_ros_actuation.sh \
  serial_device:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
```

NMPC active dùng cho held/rolling gate có evidence; hardware config mặc định
đã là active cho test có kiểm soát:

```bash
ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/install/race_cars/share/race_cars/config/nmpc_hardware.yaml \
  -p shadow_mode:=false
```

## 5. Gate còn lại

Không làm lại các gate đã PASS ở mục 3. Gate 1 đã PASS; chỉ kiểm tra lại
actuation DISARMED nếu runtime được khởi động lại. Thứ tự tiếp theo:

1. Gate 2 v2: PASS ở bước model-fit. Không chạy lại Gate 2 chỉ vì
   `stop_duration_s` trong session thực tế là `1,0 s`; nếu cần xác nhận riêng
   hành vi dừng `2,5 s`, tạo evidence bổ sung, không thay thế report PASS hiện tại.

2. Active NMPC pre-arm: **PASS** tại
   `evidence/20260715-095303_gate3_default_model_prearm_20260715`; khi chạy lại
   vẫn kiểm tra
   `/line/status=line_detected`, `/lane/reference` có data,
   `/nmpc/status=ACTIVE_VALID`, `/control/nmpc.valid=true`, publisher gate sạch.

3. Held active test: chỉ sau Gate 3 PASS; xe bị giữ chắc, E-stop sẵn sàng,
   arm 5–10 s, record bag, disarm về `[1500,1550]`.

4. Rolling slow: chỉ sau held PASS, chạy thấp tốc trong khu vực an toàn, có
   người giám sát/E-stop, record bag và đóng session bằng evidence.

## 6. Safety

- Không dừng process do session khác tạo.
- Không dùng `pkill` hoặc `killall`.
- Không flash firmware trong `archive/`; dùng source dưới `firmware/` và chạy
  `bash scripts/verify_firmware_source.sh`.
- Chỉ `rc_actuation` được phép publish `/mcu/actuator_cmd`.
- Không chạy rolling tự hành nếu `/control/nmpc.valid` không true, publisher
  gate không sạch, E-stop không sẵn sàng, hoặc chưa có evidence held PASS.
- Gate 2 v2 là ngoại lệ có kiểm soát: rolling source riêng được phép publish
  `/control/nmpc` để thu model, nhưng không được chạy đồng thời với NMPC.

## 7. Bản đồ tài liệu

- [`README.md`](README.md): cửa vào và lệnh nhanh.
- [`docs/vi/README.md`](docs/vi/README.md): mục lục tài liệu vận hành.
- [`docs/vi/operator_runbook.md`](docs/vi/operator_runbook.md): runbook gate.
- [`docs/vi/step03_nmpc.md`](docs/vi/step03_nmpc.md): NMPC shadow/active gate.
- [`docs/vi/step06_safe_corridor.md`](docs/vi/step06_safe_corridor.md):
  vật cản LiDAR, hành lang an toàn, FSM, NMPC ràng buộc hành lang, safety monitor.
- [`docs/vi/step04_microros_actuation.md`](docs/vi/step04_microros_actuation.md):
  micro-ROS/actuation fail-safe.
- [`docs/vi/run_lane_detection.md`](docs/vi/run_lane_detection.md): camera/lane.
- [`docs/vi/archive.md`](docs/vi/archive.md): mục lục material cũ.
