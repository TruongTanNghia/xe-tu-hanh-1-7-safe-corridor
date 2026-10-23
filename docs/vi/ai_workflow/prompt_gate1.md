# Prompt cho chat mới — tiếp tục Gate 3

> Tài liệu này là lịch sử chuyển tiếp. Snapshot và trạng thái hiện hành nằm ở
> [`project_state.md`](../../../project_state.md); không dùng các giá trị cũ trong
> prompt này để suy ra config runtime hiện tại. Gate 3 và Gate 4 đã PASS; Gate 5
> test 1–3 ngày 2026-07-16 chưa PASS.

Copy toàn bộ prompt dưới đây vào chat mới.

````text
Bạn đang làm việc trong repo:
`/home/ac/main/1_projects/autonomous_car_hardware_platform_v2`

Hãy đọc trước:
1. `project_state.md`
2. `README.md`
3. `docs/vi/operator_runbook.md`
4. `docs/vi/step03_nmpc.md`
5. `docs/vi/step04_microros_actuation.md`
6. `docs/vi/test_checklist.md`

Mục tiêu của phiên này:
- Hướng dẫn tôi test các safety gate còn lại để tiến tới active NMPC/rolling.
- Gate 1 và Gate 2 model-fit đã PASS; tiếp tục Gate 3, không làm lại các gate
  đã PASS nếu runtime chưa đổi.
- Không làm lại gate đã PASS trong `project_state.md`.
- Mỗi gate chỉ chuyển tiếp khi output/evidence đủ PASS.
- Nếu output không đạt, dừng ở gate đó và hướng dẫn debug đúng nguyên nhân.

Quy tắc an toàn bắt buộc:
- Không dùng `pkill`, `killall`, không dừng process không do phiên này tạo.
- Nếu cần dừng, chỉ hướng dẫn Ctrl+C đúng terminal mà tôi vừa mở.
- Không sửa code/config nếu chưa phân tích nguyên nhân và tôi chưa đồng ý.
- Không hướng dẫn rolling tự hành nếu pre-arm, held, E-stop, actuator state và
  publisher gate chưa PASS. Gate 2 v2 là rolling calibration có kiểm soát,
  không phải rolling tự hành.
- Luôn record evidence bằng `scripts/evidence` trước các bước arm/held/rolling.
- Nếu có bất thường: disarm hoặc E-stop.

Trạng thái repo hiện tại cần nhớ:
- `project_state.md` là nguồn trạng thái canonical, snapshot 2026-07-14.
- Full-loop shadow + micro-ROS/actuation DISARMED đã PASS:
  `evidence/20260711-172406_full_loop_actuation_disarmed`.
- Session DISARMED PASS có các invariant:
  - `/line/status=line_detected` toàn bộ 1.682 mẫu;
  - fusion evaluator PASS trong session lịch sử;
  - runtime LiDAR/RF2O hiện khoảng 6,4 Hz và filtered khoảng 6 Hz khi không có
    nguồn IMU; đây là tần số nguồn `vx`, không phải lỗi cần ép lên 50 Hz;
  - LiDAR snapshot PASS;
  - `/nmpc_cmd` luôn `[0,0]`;
  - `/control/nmpc.valid` không bao giờ true;
  - `/actuation/status` luôn `state=DISARMED`;
  - `/mcu/arm` không bao giờ true;
  - `/mcu/actuator_cmd` luôn `(1500,1550)`;
  - `/mcu/actuator_state` luôn `[1500,1550,29]`;
  - publisher duy nhất của `/mcu/actuator_cmd` là `actuation_manager`.
- Arm rejection khi `steering_calibrated=false` đã PASS trước khi steering gate
  được bật.
- Runtime config hiện tại:
  - `ros2_ws/src/rc_actuation/config/actuation_hardware.yaml`:
    `steering_calibrated=true`, `esc_calibrated=true`, `control_rate_hz=50.0`
  - `ros2_ws/src/race_cars/config/nmpc_hardware.yaml`:
    `shadow_mode=true`, `model_calibrated=false`
- Source config và installed config đã được kiểm tra là khớp.
- Runtime RF2O là `/v2_rf2o_laser_odometry`; `/odometry/rf2o_raw` có đúng một
  publisher. Duplicate node cũ đã được sửa bằng cách bỏ global node remap.
- `/bno055/imu` hiện có 0 publisher runtime. Không tuyên bố full BNO/EKF fusion;
  Gate 2 dọc chỉ dùng filtered `vx` khi odometry fresh.
- Gate 2 wheels-up session lịch sử FAIL vì LiDAR tắt nên CSV có 0 mẫu; session
  đã được dọn khỏi Pi. Không dùng kết quả đó để fit.
- Gate 2 v2 rolling session lịch sử
  `evidence/20260713-171843_longitudinal_fit_rolling_v2_60s` bị giữ làm
  evidence chẩn đoán: sequence abort ở `vx≈0,400 m/s`, chỉ có 15 transition
  hợp lệ và filtered khoảng 3,65 Hz.
- Phân tích bag xác nhận PWM 1580 làm RF2O raw đạt khoảng 0,474 m/s và delta
  pose cũng cùng cỡ; đây không phải chỉ là EKF spike. Đã sửa EKF
  `sensor_timeout` 0,3→0,15 s; build/test `6/6` PASS. Cần restart xác nhận
  runtime filtered vượt 6 Hz và hiệu chỉnh PWM/ESC sàn, không nới speed limit.
- Probe `1595 us` đã hoàn tất trong
  `evidence/20260714-095327_longitudinal_probe_1595_hold6s`: vận tốc plateau
  khoảng `0,516 m/s`, đỉnh `0,538 m/s`. Guard Gate 2 đã chốt ở
  `max_speed_mps=0,6 m/s`; guard tạm `1,0 m/s` chỉ áp dụng cho probe cũ.
- Gate 2 v2 model-fit đã PASS trong
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
  184 armed transitions, đủ ba PWM, actuation/control khoảng 50 Hz, filtered
  khoảng 6,17 Hz, held-out RMSE `0,007438 m/s`. Candidate YAML chưa merge.
  Review lưu ý process thực tế dùng `stop_duration_s:=1.0` dù tên scenario là
  `stop2p5`; candidate `model.cr3=20` chạm bound trên của fitter.
- `config/vehicle_1_7.yaml` hiện ghi:
  - LiDAR/IMU extrinsics: `measured_ccw_verified`
  - steering: `enabled_operator_confirmed_wheels_up`
  - NMPC longitudinal model: `blocked_requires_step_response`
- Candidate model dọc hiện có tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s/plots/longitudinal_model/nmpc_model_candidate.yaml`.
  Config mặc định vẫn giữ `model_calibrated=false` cho tới khi candidate được
  review/merge có chủ đích.

Lệnh runtime canonical:
Mỗi terminal:
```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
```

Nếu chạy NMPC/acados:
```bash
source scripts/acados_env.sh
```

Sensors không kèm NMPC:
```bash
ros2 launch rc_bringup shadow_full.launch.py start_nmpc:=false
```

micro-ROS + actuation manager:
```bash
bash scripts/run_micro_ros_actuation.sh \
  serial_device:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
```

NMPC active chỉ dùng cho gate có kiểm soát và có evidence:
```bash
ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/install/race_cars/share/race_cars/config/nmpc_hardware.yaml \
  -p shadow_mode:=false \
  -p model_calibrated:=true
```

Stop an toàn:
```bash
ros2 service call /actuation/arm std_srvs/srv/SetBool "{data: false}"
ros2 service call /actuation/estop std_srvs/srv/Trigger "{}"
```

Các gate hiện tại:
1. Gate 1 — actuation config/stop state: PASS.
2. Gate 2 v2 — rolling model-fit trên sàn, LiDAR bật: **PASS**. Evidence
   chính thức dùng ba mức PWM `1580/1585/1590 us`, guard `0,6 m/s`, actuation
   khoảng 50 Hz và held-out RMSE `0,007438 m/s`.
3. Gate 3 — active NMPC pre-arm.
4. Gate 4 — held active test.
5. Gate 5 — rolling slow tự hành.

Bước đang làm là review candidate và chuẩn bị Gate 3. Không chạy lại Gate 2
trừ khi cần evidence riêng cho stop `2,5s`; không dùng
`run_esc_step_calibration.sh`/`/calibration/start` trên sàn:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

SESSION=$(bash scripts/evidence/new_session.sh \
  --scenario longitudinal_fit_rolling_v2)
bash scripts/evidence/observe.sh --session "$SESSION" --duration 10
```

Không tạo lại rolling session để đánh giá lại Gate 2 nếu không có yêu cầu
riêng về stop `2,5s`. Runbook lệnh đầy đủ hiện nằm ở
`docs/vi/command_teminal.md`; Gate 2 đã có evidence và report PASS.

Việc tiếp theo là review candidate YAML, giữ `model_calibrated=false` trong
config mặc định cho tới khi có quyết định merge có chủ đích, rồi chuẩn bị Gate
3 active NMPC pre-arm. Không tự bật active NMPC chỉ bằng cách truyền
`model_calibrated:=true` nếu các điều kiện lane/reference, publisher gate,
E-stop và evidence pre-arm chưa PASS.
````
