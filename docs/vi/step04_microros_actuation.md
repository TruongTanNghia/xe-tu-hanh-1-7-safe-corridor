# Bước 4 — micro-ROS và actuation fail-safe

## Trạng thái hiện tại — 2026-07-16

Full-loop shadow + micro-ROS/actuation DISARMED đã PASS tại
`evidence/20260711-172406_full_loop_actuation_disarmed`. Trong session đó
`/mcu/actuator_cmd` luôn `(1500,1550)`, `/mcu/arm` luôn false,
`/mcu/actuator_state` luôn `[1500,1550,29]`, và publisher duy nhất của
`/mcu/actuator_cmd` là `actuation_manager`.

Config hiện tại đã bật:

```yaml
steering_calibrated: true
esc_calibrated: true
```

Điều này chỉ mở gate cho các test có kiểm soát. Candidate NMPC đã được review
và `model_calibrated=true`; hardware config mặc định dùng active cho các phiên
test có kiểm soát. Không coi config này là duyệt rolling tự hành.

Trong Gate 5 test 3, actuation kết thúc đúng safe output sau khi upstream phát
fault: không reverse, servo nằm trong `1150..1850 us`, ESC trong
`1550..1600 us`, và trạng thái cuối là `DISARMED`, `servo=1500`, `esc=1550`.
Điều này xác nhận hành vi safe-stop của tầng actuation trong lượt đó, nhưng
không biến Gate 5 thành PASS.

## 1. Chuẩn bị

- Khi chỉ bringup/kiểm tra output: ngắt nguồn động lực ESC hoặc giữ xe an toàn.

### Diagnostic servo-only

`actuation_manager` hiện chỉ có contract chung cho servo và ESC, chưa có tham
số `servo_only`. Vì vậy phiên chẩn đoán image processing phải vô hiệu hóa
motor/ESC bằng biện pháp vật lý đã được xác nhận; không dùng
`esc_calibrated=false` như workaround vì nó làm arm bị từ chối. Trong phiên đó
ESC phải giữ `1550`, chỉ servo được phép thay đổi trong `1150..1850`. Xem
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).
- Với Gate 2 v2: xe đặt trên sàn, LiDAR bật, E-stop trong tầm tay và người vận
  hành kiểm soát hướng/chướng ngại. Không dùng wheels-up source cho test này.
- Không chạy `shadow_full` và calibration source nếu chưa kiểm tra publisher.
- Firmware được duyệt nằm ở
  `firmware/stm32/nucleo_f411re_microros`, không phải `archive/`.

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/preflight.sh --hardware
bash scripts/verify_firmware_source.sh
ls -l /dev/serial/by-id
```

Build/link firmware trong STM32CubeIDE phải `0 errors` trước khi flash. Hướng
dẫn firmware chi tiết nằm trong `firmware/.../README_vi.md`.

## 2. Lệnh chạy

Terminal 1, thay path bằng output thực của `ls`:

```bash
bash scripts/run_micro_ros_actuation.sh \
  serial_device:=/dev/serial/by-id/<STM32_ID>
```

Wrapper ưu tiên `micro_ros_agent` đã build trong V2; workspace micro-ROS cũ chỉ
là compatibility fallback. Node luôn khởi động DISARMED. Không gọi arm nếu
calibration YAML còn false, hoặc nếu `/control/nmpc` không có command valid ở
gate active.

## 3. Quan sát

```bash
ros2 topic hz /mcu/status
ros2 topic echo /mcu/actuator_state
ros2 topic echo /actuation/status
ros2 topic info -v /mcu/actuator_cmd
ros2 service list | grep /actuation
```

`/mcu/actuator_cmd` phải có đúng một publisher là `actuation_manager`.
State disarmed phải có `[1500,1550,flags]`.

Gate 2 v2 dùng source và service riêng:

```bash
bash scripts/run_esc_rolling_calibration.sh --rolling-slow-confirmed
ros2 topic echo /calibration/rolling_step_status --once
ros2 service call /calibration/rolling_start std_srvs/srv/Trigger "{}"
```

`rolling_start` chỉ được gọi sau khi record evidence, source báo `READY_STOP`,
publisher gate sạch và người vận hành đã arm có chủ đích. Nếu thấy bất thường,
gọi `/actuation/arm` với `data: false` và E-stop.

## 4. Xuất dữ liệu

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario microros_g1)
bash scripts/evidence/observe.sh --session "$SESSION" --duration 10
bash scripts/evidence/record.sh --session "$SESSION" --duration 30 -- \
  /mcu/status /mcu/actuator_state /mcu/actuator_cmd /mcu/arm \
  /actuation/status /control/nmpc
```

Thử reconnect bằng cách chỉ dừng agent do chính Terminal 1 khởi động rồi chạy
lại launch. Không rút nguồn motor trong test logic.

## 5. Đánh giá

G1 PASS khi heartbeat/state xuất hiện, reconnect được và output luôn stop.
Session `20260711-172406_full_loop_actuation_disarmed` đã PASS phần full-loop
DISARMED. Gate 2 v2 chỉ chạy sau khi LiDAR/RF2O/EKF có dữ liệu fresh và xác
nhận hiện trường:

- arm thiếu command, MCU heartbeat hoặc calibration phải bị từ chối;
- mất command/heartbeat/agent trong tối đa timeout phải DISARM;
- E-stop latch cho tới khi gọi clear;
- servo clamp `1150..1850`, ESC `1550` hoặc `1580..1600`;
- Ctrl+C kết thúc sạch, không traceback; nếu ROS context đã invalid trước khi
  stop burst hoàn tất thì timeout ứng dụng phía MCU vẫn phải đưa output về
  stop trong tối đa 250 ms khi firmware loop còn chạy.
- Hardware IWDG hiện đang tắt trong firmware bring-up, nên không được tuyên
  bố đã kiểm chứng watchdog reset MCU ở gate này.

Firmware cũ trong archive không đạt gate này.

## 6. Lưu bằng chứng

```bash
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Chụp thêm Build Console STM32, video Gate 2 v2 và ảnh nguồn motor bị ngắt.
Không đổi các cờ calibration trong tương lai nếu thiếu artifact tương ứng.
