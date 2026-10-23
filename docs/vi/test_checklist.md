# Checklist ký xác nhận

## Trạng thái hiện tại — 2026-07-16

- Full-loop shadow + actuation DISARMED: PASS tại
  `evidence/20260711-172406_full_loop_actuation_disarmed`.
- `steering_calibrated=true`, `esc_calibrated=true` trong config actuation.
- `model_calibrated=true` trong config NMPC mặc định sau review candidate;
  `shadow_mode=false` dùng cho các phiên test có kiểm soát. Profile
  `shadow_full.launch.py` vẫn ép shadow.
- Runtime RF2O hiện là `/v2_rf2o_laser_odometry`, không còn duplicate node.
- `/bno055/imu` hiện chưa có publisher runtime; chưa ký full IMU/EKF fusion.
- Gate 2 v2 model-fit đã PASS tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
  184 armed transitions, đủ ba PWM, held-out RMSE `0,007438 m/s`, bag `59,4s`.
- Runtime evidence xác nhận actuation/control khoảng `50 Hz`, filtered khoảng
  `6,17 Hz`, armed peak `0,381 m/s` dưới guard `0,6 m/s`.
- Review note: process thực tế dùng `stop_duration_s:=1.0`, dù scenario ghi
  `stop2p5`; hành vi stop `2,5s` chưa có evidence riêng.
- Gate 5 test 1, 2 và 3 ngày 2026-07-16 đều chưa PASS. Test 3 dùng exposure
  `4000 us`, active khoảng `21,5 s`, filtered peak `0,504 m/s`, có
  `lane_invalid`, `cte_outside_safe_bound` và `solver_failed`, nên chưa đạt
  ba vòng liên tiếp. Video: [`evidence/test3_gate5.mp4`](../../evidence/test3_gate5.mp4).
- Lệnh chạy thủ công theo từng terminal đã tách tại
  [`gate5_manual_test.md`](gate5_manual_test.md).
- Checklist dưới đây là checklist gate tổng quát; không làm lại mục đã có
  evidence PASS trừ khi source/config/runtime đã đổi.

## G0 — không cấp động lực

- [ ] `scripts/preflight.sh --hardware` PASS.
- [ ] `scripts/build.sh` và `scripts/test.sh` PASS.
- [ ] Lane raw/debug/fitting đúng đoạn thẳng và cua.
- [ ] `/scan` có valid points; web viewer/SVG và rosbag đều mở được.
- [ ] Fusion đứng yên đạt mean/P95/spike gate.
- [ ] Quay CCW cho `gyro.z > 0`.
- [ ] NMPC benchmark P99 < 50 ms; `/nmpc_cmd=[0,0]`.

## Gate 1 — MCU, actuation và servo DISARMED

- [ ] STM32CubeIDE build firmware mới `0 errors`.
- [ ] `/mcu/status` và `/mcu/actuator_state` đúng.
- [ ] Agent restart thì MCU reconnect, output vẫn stop.
- [ ] Chỉ một publisher `/mcu/actuator_cmd`.
- [ ] Servo trái/giữa/phải đúng chiều và không vượt clamp.
- [ ] Mất command/arm đưa servo giữa, ESC stop.

## Gate 2 v2 — model dọc, xe trên sàn

- [x] `/scan`, `/odometry/rf2o_raw`, `/odometry/filtered` có dữ liệu fresh.
- [x] `/odometry/rf2o_raw` chỉ có một publisher `/v2_rf2o_laser_odometry`.
- [x] Evidence/rosbag và longitudinal CSV bắt đầu trước khi arm.
- [x] Chạy rolling source riêng, không chạy
      `run_esc_step_calibration.sh`.
- [x] `rolling_step_source` là publisher duy nhất `/control/nmpc`.
- [x] Probe `1595 us` hoàn tất trong guard tạm thời bảo thủ; vận tốc được review
      và guard chính thức được chốt với biên an toàn.
- [x] Guard chính thức không bằng đúng vận tốc đo được; guard Gate 2 là
      `0,6 m/s`, còn `TEMP_GUARD=1,0 m/s` chỉ dùng cho probe.
- [x] Các lượt fit `1580/1585/1590 us` đều nằm dưới guard chính thức.
- [x] Đối chiếu raw RF2O với delta pose: cả hai cùng xác nhận tốc độ khoảng
      `0,46–0,50 m/s` tại PWM 1580; không tăng giới hạn tốc độ để bỏ qua abort.
- [x] Đã xác định cadence thấp do EKF `sensor_timeout=0,3 s`; đã sửa tiếp về
      `0,15 s`, build/test `6/6` PASS; runtime evidence mới xác nhận khoảng
      `6,17 Hz`.
- [x] `/odometry/filtered` runtime thật khoảng `6,17 Hz`.
- [x] CSV có ít nhất 50 armed transitions hợp lệ (`184`).
- [x] CSV có đủ các mức PWM/effort `1580/1585/1590`.
- [x] Dữ liệu fit được lưu trong cùng evidence session.
- [x] Held-out velocity RMSE `≤ 0,04 m/s`; report và candidate YAML được lưu.
- [x] Candidate được review bằng RMSE/bound analysis và Gate 3 pre-arm trước
      khi đổi `model_calibrated=true`.

## Gate 3/Gate 4 — active pre-arm rồi xe bị giữ

- [x] Gate 3 active pre-arm PASS tại
      `evidence/20260715-095303_gate3_default_model_prearm_20260715`; motor
      vẫn DISARMED, `/mcu/arm=false` toàn bộ session.
- [x] Gate 4 held active PASS tại
      `evidence/20260716-104744_gate4_held_active_manual_exposure_20260716`;
      các fault/reconnect evidence liên quan nằm trong các session Gate 4 ngày
      2026-07-16.
- [ ] Arm bị chặn khi thiếu một precondition.
- [ ] ESC ramp đúng `1550 → 1580..1600`.
- [ ] E-stop, Ctrl+C sạch và mất agent đều đưa output về `1550`.
- [ ] Mất `/mcu/actuator_cmd` hoặc `/mcu/arm` đưa output về `1550` trong
      timeout ứng dụng 250 ms khi vòng lặp firmware vẫn chạy.
- [ ] Hardware IWDG: **không áp dụng trong cấu hình hiện tại**; không được
      ghi nhận như một điều kiện PASS của Gate 4.
- [ ] Không reverse.

## Gate 5 — rolling slow tự hành

- [ ] Extrinsic LiDAR/IMU đã đo và cập nhật bằng evidence.
- [ ] Fusion chạy đều đạt stddev/error gate.
- [ ] Fault lane/odom/controller/MCU đều stop.
- [x] Active held test PASS trước rolling slow tại
      `evidence/20260716-104744_gate4_held_active_manual_exposure_20260716`.
- [ ] Khu vực an toàn, có E-stop và người giám sát.
- [ ] Target 0,6 m/s theo source research, không vượt biên.
- [ ] Ba vòng liên tiếp PASS.
- [ ] Session có rosbag, report, video và SHA256SUMS.

## Diagnostic trước khi chạy lại Gate 5

- [ ] Motor/ESC đã vô hiệu hóa vật lý; E-stop và người giám sát sẵn sàng.
- [ ] Không dùng `esc_calibrated=false` để giả lập servo-only; runtime hiện chưa
      có `servo_only` parameter.
- [ ] Rosbag bắt đầu trước khi đẩy xe, có raw/debug/fitting image.
- [ ] Có dữ liệu đồng bộ `/line/status`, confidence, `cte`, `kappa`, `s`,
      `heading`, `/lane/reference` và servo PWM.
- [ ] ESC giữ `1550` toàn bộ phiên; servo nằm trong `1150..1850`.
- [ ] Có đánh dấu thời gian cho straight, enter-curve, mid-curve, exit-curve.
- [ ] Đối chiếu ảnh, output perception và servo theo cùng timestamp trước khi
      quy kết lỗi cho image processing hoặc NMPC.
- [ ] Diagnostic PASS không được ghi nhận là Gate 5 autonomous PASS.

Người test: __________  Ngày: __________  Gate cao nhất PASS: __________
