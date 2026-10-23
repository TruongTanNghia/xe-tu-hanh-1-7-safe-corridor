# Gate 5 — lệnh chạy thủ công

Nếu chỉ cần quan sát nhanh qua bốn panel mà không ghi dữ liệu, dùng
[`gate5_manual_live_no_record.md`](gate5_manual_live_no_record.md). Lượt đó là
exploratory và không thể dùng để kết luận Gate 5 PASS.

Tài liệu này là quy trình thực hành cho rolling slow full map. Đây là runbook
thực nghiệm, không phải bằng chứng Gate 5 PASS. Gate 5 chỉ được kết luận PASS
khi có ba vòng liên tiếp, rosbag/report/video/SHA256SUMS đầy đủ và không có
fault lane, odometry, solver hoặc actuation chưa được giải thích.

## Diagnostic trước Gate 5 — image processing + servo-only

Trước khi chạy autonomous full map, dùng
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md)
để thực hiện phiên đẩy xe bằng tay và ghi ảnh. Mục tiêu là đối chiếu cùng
timestamp giữa raw/debug/fitting image, `cte`, `kappa`, `s`, `heading`, lane
reference và PWM servo. Đây là diagnostic, không phải Gate 5 PASS.

`actuation_manager` hiện chưa có `servo_only`; `esc_calibrated=false` sẽ chặn
arm toàn bộ chứ không tạo servo-only. Chỉ tiến hành khi motor/ESC đã được vô
hiệu hóa vật lý hoặc đã có test mode riêng được review. Trong toàn phiên ESC
phải giữ `1550`, còn servo mới được phép thay đổi trong `1150..1850`.

Trạng thái sau test 3 ngày 2026-07-16:

- exposure `4000 us` tốt hơn `8000 us` ở đoạn đầu;
- xe active khoảng `21,5 s`, tốc độ filtered tối đa `0,504 m/s`;
- vẫn có `lane_invalid`, `cte_outside_safe_bound` và safe-stop;
- Gate 5 chưa PASS.

Lượt camera-capture tiếp theo đã lưu tại
`evidence/20260716-123801_gate5_camera_capture_exposure4000_20260716`: bag có
ảnh liên tục nhưng arm tự chuyển lại DISARMED trước khi xe chạy. Không arm lại
trong cùng session; phải phân tích nguyên nhân safe-stop trước.

Lượt Gate 5 cuối ngày 2026-07-16 tại
`evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716` cũng chưa
PASS: armed khoảng `30,8 s`, có `solver_failed`, `lane_invalid` và safe-stop.
Không dùng evaluator expected-speed trên toàn bag hỗn hợp active/disarmed như
metric active-only.

## An toàn trước khi mở terminal

- Xe ở đúng vị trí xuất phát trên full map; khu vực trống, E-stop trong tầm tay,
  người giám sát sẵn sàng.
- Không dùng `pkill`, `killall` hoặc `ros2 daemon stop`.
- Chỉ Ctrl+C các process do chính phiên này khởi động.
- Bắt đầu rosbag trước khi arm.
- Nếu `/nmpc/status` không `ACTIVE_VALID`, `/control/nmpc.valid` false,
  lane/odom stale, hoặc tốc độ vượt `0,60 m/s`: disarm ngay.
- Không gọi arm trong bước pre-arm.

## Terminal 0 — setup và kiểm tra một lần

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

SERIAL_DEVICE=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
test -e "$SERIAL_DEVICE" && echo "Serial OK" || echo "Serial NOT FOUND"
readlink -f "$SERIAL_DEVICE"
```

Kỳ vọng: serial tồn tại và thường trỏ tới `/dev/ttyACM0`. Nếu `Serial NOT FOUND`,
dừng tại đây, kiểm tra USB/udev; không arm.

## Terminal 1 — camera/lane, chạy lâu dài

Dùng exposure thấp cho điều kiện sáng gắt. Giá trị test 3 đã dùng là `4000 us`;
giữ analogue gain `1.0` để thay đổi một biến trước.

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

ros2 launch hardware_camera lane_detection.launch.py \
  manual_controls:=true \
  exposure_time_us:=4000 \
  analogue_gain:=1.0
```

Kỳ vọng: log có `Manual camera controls enabled: ExposureTime=4000 us`,
`bands=12/12`, CTE/heading ổn định. Có thể so sánh `3000` hoặc `5000 us` ở
phiên riêng; không đổi exposure trong khi đang record/arm.

## Terminal 2 — LiDAR/RF2O/EKF, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

bash scripts/run_fusion.sh start_imu:=false
```

`start_imu:=false` là chủ ý vì `/bno055/imu` không có publisher runtime.
Kỳ vọng LiDAR health `OK`, RF2O và `/odometry/filtered` có dữ liệu mới.

## Terminal 3 — lane adapter, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

ros2 run race_cars lane_reference_adapter
```

## Terminal 4 — NMPC active, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/install/race_cars/share/race_cars/config/nmpc_hardware.yaml \
  -p shadow_mode:=false
```

## Terminal 5 — micro-ROS actuation, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

bash scripts/run_micro_ros_actuation.sh \
  serial_device:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
```

Kỳ vọng ban đầu: `Actuation starts DISARMED`, MCU heartbeat xuất hiện,
`servo=1500`, `esc=1550`. Nếu agent báo `Serial port not found`, Ctrl+C terminal
này, kiểm tra lại device rồi khởi động lại; không dùng port đoán khác.

## Terminal 6 — pre-arm checks, mỗi lệnh chạy một lần

Chỉ tiếp tục khi tất cả kết quả đạt:

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

ros2 topic echo /line/status --once
ros2 topic echo /lane/reference --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /nmpc/status --once
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
ros2 topic info -v /control/nmpc
ros2 topic info -v /mcu/actuator_cmd
```

Điều kiện bắt buộc:

- `/line/status`: `line_detected`;
- `/lane/reference`: `status=1`, confidence hợp lệ và timestamp mới;
- `/odometry/filtered`: timestamp mới, tốc độ gần `0` khi xe đứng yên;
- `/nmpc/status`: `ACTIVE_VALID`;
- `/control/nmpc`: đúng một publisher `racecar_nmpc_node`;
- `/mcu/actuator_cmd`: đúng một publisher `actuation_manager`;
- `/actuation/status`: `DISARMED`, `servo_us=1500`, `esc_us=1550`;
- `/mcu/actuator_state`: `[1500,1550,flags]`.

## Terminal 7 — tạo session và ghi rosbag trước arm

Chạy một lần để tạo session, sau đó dùng đúng đường dẫn được in ra:

```bash
SESSION=$(bash scripts/evidence/new_session.sh \
  --scenario gate5_full_map_rolling_exposure4000)
printf '%s\n' "$SESSION"
```

Chạy recorder ở terminal riêng và để chạy lâu dài:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest

bash scripts/evidence/record.sh --session "$SESSION" --duration 180 -- \
  /camera/front_camera/image_raw /line/debug_image /line/fitting_image \
  /scan /line/status /lane/reference /odometry/rf2o_raw /odometry/rf2o \
  /odometry/filtered /nmpc/status /nmpc/solve_time_ms /control/nmpc \
  /mcu/status /mcu/actuator_state /mcu/actuator_cmd /mcu/arm \
  /actuation/status
```

Không arm trước khi recorder in `Recording PID ...`.

Ba topic ảnh được ghi liên tục để phân tích sau:

- `/camera/front_camera/image_raw`: ảnh đầu vào thực tế, dùng để kiểm tra
  ánh sáng, motion blur và vùng nhìn thấy đường;
- `/line/debug_image`: scan bands/centroid mà processor sử dụng;
- `/line/fitting_image`: đường fit, biên lane và vanishing point.

Ảnh `640x360 bgr8` ở khoảng 30 FPS có thể làm rosbag lớn nhanh. Trước khi
chạy, kiểm tra `df -h`; với lượt đầu nên dùng thời lượng `60–90 s` thay vì
`180 s` nếu dung lượng trống không đủ. Không dùng ảnh rời rạc thiếu timestamp
để thay thế rosbag.

## Terminal 8 — quay video

Quay toàn bộ lượt chạy, gồm vị trí xuất phát, chuyển làn, cua gắt và thời điểm
safe-stop. Sau khi dừng xe, lưu video vào session:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest
mkdir -p "$SESSION/video"
cp /duong/dan/video_gate5.mp4 "$SESSION/video/gate5.mp4"
```

Nếu video nằm ở ngoài repo, phải ghi rõ đường dẫn trong `report.md`; video không
được coi là đã lưu chỉ vì tồn tại ở một thư mục khác.

## Terminal 9 — arm và theo dõi

Chỉ chạy sau khi rosbag đang ghi và mọi pre-arm check PASS:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: true}"
```

Theo dõi bằng các terminal one-shot hoặc terminal riêng:

```bash
ros2 topic echo /odometry/filtered \
  --field twist.twist.linear.x
ros2 topic echo /nmpc/status --field data
ros2 topic echo /actuation/status --field data
ros2 topic echo /mcu/actuator_state
```

Không gọi arm lại trong cùng session sau `command_invalid`, `lane_invalid`,
`solver_failed` hoặc `cte_outside_safe_bound`; disarm, giữ evidence và phân tích.

## Terminal 10 — dừng an toàn và đóng session

Disarm rõ ràng sau khi xe dừng hoặc khi có bất thường:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
```

Kỳ vọng cuối: `DISARMED`, `servo=1500`, `esc=1550`. Sau đó Ctrl+C lần lượt các
terminal 5, 4, 3, 2, 1; chỉ dừng process do session này tạo.

Đánh giá bag và đóng session:

```bash
ros2 bag info "$SESSION/rosbag/<bag_name>"
bash scripts/evidence/evaluate.sh --session "$SESSION" \
  --mode fusion --bag "$SESSION/rosbag/<bag_name>" --expected-speed 0.6
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Nếu thiếu ba vòng, thiếu video, evaluator FAIL hoặc có fault chưa giải thích,
dùng `--result FAIL` hoặc `--result INCOMPLETE`; không đổi kết quả bằng tay.
