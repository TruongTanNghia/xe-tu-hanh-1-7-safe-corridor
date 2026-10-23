# Command terminal — Runbook lệnh phần cứng V2

Tài liệu này là cheat sheet cho Gate 2 v2. Mỗi nhóm lệnh chạy trong một
terminal riêng. Không dùng `pkill`, `killall` hoặc dừng process không do mình
tạo.

Trạng thái hiện tại:

- Gate 2 model-fit: **PASS**;
- evidence chính thức:
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`;
- held-out RMSE: `0.007438 m/s`;
- actuation/control khoảng `50 Hz`, filtered khoảng `6.17 Hz`;
- guard Gate 2: `max_speed_mps=0.6 m/s`;
- candidate YAML đã được review/merge; Gate 3 active pre-arm đã PASS. Gate 4
  held active đã PASS theo trạng thái dự án; Gate 5 rolling slow chưa PASS sau
  ba lượt thử full map ngày 2026-07-16.

## Gate 5 — lệnh chạy thủ công

Trước Gate 5 autonomous, phiên diagnostic mới cần tách image processing khỏi
NMPC bằng cách vô hiệu hóa motor/ESC vật lý, đẩy xe bằng tay và chỉ theo dõi
servo. Prompt đầy đủ nằm tại
[`ai_workflow/prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).
Phải record ảnh cùng `cte`, `kappa`, `s`, `heading`; ESC phải giữ `1550`.

Không dùng phần lệnh Gate 2 để chạy rolling Gate 5. Quy trình đầy đủ, gồm
camera manual exposure `4000 us`, fusion không BNO055, active NMPC, serial
STM32 đúng, pre-arm checks, rosbag trước arm và dừng an toàn, nằm ở
[`gate5_manual_test.md`](gate5_manual_test.md).

Test 3 tham chiếu dùng:

```bash
ros2 launch hardware_camera lane_detection.launch.py \
  manual_controls:=true exposure_time_us:=4000 analogue_gain:=1.0
```

Đây là giá trị thực nghiệm cho điều kiện sáng ngày 2026-07-16, không phải
camera default. Không kết luận Gate 5 PASS nếu chưa đủ ba vòng liên tiếp.

Lưu ý review: process snapshot của evidence trên thực tế dùng
`stop_duration_s:=1.0`, dù tên scenario ghi `stop2p5`. Không dùng session đó
để kết luận riêng về hành vi dừng `2.5s`.

## 0. Quy ước chung

Chạy ở đầu mỗi terminal:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
```

Serial STM32 hiện tại:

```bash
SERIAL_DEVICE=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
test -e "$SERIAL_DEVICE" && echo "Serial OK" || echo "Serial NOT FOUND"
readlink -f "$SERIAL_DEVICE"
```

Session hiện hành luôn có thể tham chiếu bằng symlink:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest
```

`evidence/latest` trỏ tới session mới nhất vừa tạo.

## 1. Kiểm tra process trước khi khởi động

Nếu một pipeline đã chạy, giữ pipeline đó và không khởi động bản thứ hai.

```bash
ps -eo pid,ppid,lstart,args | rg \
  'micro_ros_agent|actuation_manager|fusion_hardware|rf2o_laser_odometry|sllidar|ekf_filter|rolling_step_source'
```

Khi cần dừng, nhấn `Ctrl+C` đúng terminal đã mở process đó.

## 2. Khởi động LiDAR + RF2O + EKF

Terminal fusion:

```bash
bash scripts/run_fusion.sh start_imu:=false
```

`start_imu:=false` dùng cho trạng thái hiện tại vì `/bno055/imu` chưa có
publisher. EKF vẫn fuse vận tốc dọc từ RF2O.

Terminal kiểm tra:

```bash
ros2 param get /ekf_filter_node sensor_timeout
timeout 10 ros2 topic hz /scan
timeout 10 ros2 topic hz /odometry/rf2o_raw
timeout 10 ros2 topic hz /odometry/rf2o
timeout 10 ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once \
  --field twist.twist.linear.x
```

Kỳ vọng hiện tại:

- `sensor_timeout` là `0.15`;
- `/scan`, RF2O và filtered khoảng `6 Hz`;
- khi xe đứng yên, filtered `linear.x` gần `0.0`.

## 3. Khởi động micro-ROS + actuation manager

Terminal actuation:

```bash
SERIAL_DEVICE=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
test -e "$SERIAL_DEVICE" || {
  echo "Không tìm thấy serial: $SERIAL_DEVICE"
  exit 1
}

bash scripts/run_micro_ros_actuation.sh \
  serial_device:="$SERIAL_DEVICE"
```

Đường dẫn đúng phải chứa đoạn `STMicroelectronics_STM32_STLink`. Nếu
`readlink -f` trả về `/dev/ttyACM0` thì board STM32 đã được nhận diện đúng.

Node này khởi động ở `DISARMED`; nó không tự arm và không flash firmware.

Kiểm tra trạng thái:

```bash
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
ros2 topic info -v /mcu/actuator_cmd
```

Trạng thái stop hợp lệ:

```text
state=DISARMED
servo_us=1500
esc_us=1550
```

## 4. Tạo session evidence mới

Chỉ tạo session một lần:

```bash
SESSION=$(bash scripts/evidence/new_session.sh \
  --scenario longitudinal_probe_1595_90s)
echo "$SESSION"
bash scripts/evidence/observe.sh \
  --session "$SESSION" \
  --duration 10
```

Sau đó các terminal khác dùng:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest
```

## 5. Probe 1595 us đã hoàn tất

Probe `1595 us` trước đó dùng guard tạm thời `1.0 m/s` và đã đo được vận tốc
plateau khoảng `0.52 m/s`, đỉnh `0.538 m/s`. Không dùng lại guard `1.0 m/s`
cho các lượt fit Gate 2.

Terminal rolling source:

```bash
ros2 run rc_actuation rolling_step_source --ros-args \
  -p safety_ack:=ROLLING_SLOW_MOTOR_POWER_CONTROLLED \
  -p "step_efforts:=[0.75]" \
  -p stop_duration_s:=1.0 \
  -p step_duration_s:=2.0 \
  -p max_speed_mps:=1.0
```

`step_efforts:=0.75` được map thành:

```text
1580 + 0.75 × 20 = 1595 us
```

Lệnh trên chỉ giữ PWM chạy trong `2s`, phù hợp với probe ngắn. Nếu cần đo
vận tốc ổn định khi giữ `1595 us`, dùng sequence dài hơn sau khi đã bảo đảm
đủ khoảng trống phía trước:

```bash
ros2 run rc_actuation rolling_step_source --ros-args \
  -p safety_ack:=ROLLING_SLOW_MOTOR_POWER_CONTROLLED \
  -p "step_efforts:=[0.75]" \
  -p stop_duration_s:=1.0 \
  -p step_duration_s:=6.0 \
  -p max_speed_mps:=1.0
```

Sequence này gồm khoảng `1s` chuẩn bị, `6s` giữ `1595 us` và `1s` dừng
cuối. Rosbag/CSV phải được khởi động trước; thời gian ghi nên đặt khoảng
`15–20s`, không có nghĩa là xe sẽ chạy suốt thời gian ghi.

Chờ source báo `READY_STOP`, sau đó kiểm tra:

```bash
ros2 topic echo /calibration/rolling_step_status --once
ros2 topic info -v /control/nmpc
```

`/control/nmpc` chỉ được có publisher `rolling_step_source`.

## 6. Bắt đầu record 90 giây

Chỉ chạy hai recorder sau khi tất cả terminal đã sẵn sàng và source đang
`READY_STOP`. Chạy mỗi lệnh ở một terminal riêng.

Terminal rosbag:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest

bash scripts/evidence/record.sh \
  --session "$SESSION" \
  --duration 90
```

Terminal longitudinal CSV:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest

bash scripts/record_longitudinal_model.sh "$SESSION" 90.0
```

`record.sh` nhận số nguyên `90`; longitudinal recorder phải nhận `90.0` vì
ROS 2 khai báo `duration_sec` là kiểu `DOUBLE`.

## 7. Arm và chạy probe

Chỉ chạy sau khi:

- rosbag đang ghi;
- CSV recorder đang chạy;
- source là `READY_STOP`;
- xe nằm trong khu vực thử;
- bánh xe không bị cản;
- E-stop nằm trong tầm tay.

Arm:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: true}"
```

Bắt đầu probe:

```bash
ros2 service call /calibration/rolling_start \
  std_srvs/srv/Trigger "{}"
```

Theo dõi trong lúc chạy:

```bash
ros2 topic echo /calibration/rolling_step_status
ros2 topic echo /odometry/filtered \
  --field twist.twist.linear.x
ros2 topic echo /actuation/status
ros2 topic echo /mcu/actuator_state
```

## 8. Kết thúc an toàn

Khi source báo `COMPLETE_STOP` hoặc `ABORTED_*`, disarm rõ ràng:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: false}"
```

Sau đó nhấn `Ctrl+C` đúng terminal rolling source. Không nhấn `Ctrl+C` hai
recorder; để chúng chạy đủ 90 giây và tự lưu dữ liệu.

Kiểm tra stop state:

```bash
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
```

## 9. Kiểm tra dữ liệu probe

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest

ls -lh "$SESSION/csv/longitudinal_samples.csv"
ros2 bag info "$SESSION/rosbag/latest"
```

In vận tốc armed lớn nhất trong CSV:

```bash
python3 - "$SESSION/csv/longitudinal_samples.csv" <<'PY'
import csv
import sys

path = sys.argv[1]
rows = list(csv.DictReader(open(path, encoding="utf-8", newline="")))
armed = [row for row in rows if int(row["armed"]) == 1]
if not armed:
    raise SystemExit("Không có armed sample")
peak = max(armed, key=lambda row: abs(float(row["vx_mps"])))
print(f"armed_samples={len(armed)}")
print(f"peak_vx_mps={float(peak['vx_mps']):.6f}")
print(f"peak_esc_us={peak['esc_us']}")
PY
```

Không chạy fit model với session probe chỉ có PWM `1595 us`.

## 10. Workflow fit ba mức PWM

Sau khi review probe và chốt `GUARD_CHINH_THUC`, model-fit dùng ba mức:

```text
1580 us → effort 0.01
1585 us → effort 0.25
1590 us → effort 0.50
```

Source tương ứng có thể chạy bằng:

```bash
GUARD_CHINH_THUC=0.6

ros2 run rc_actuation rolling_step_source --ros-args \
  -p safety_ack:=ROLLING_SLOW_MOTOR_POWER_CONTROLLED \
  -p "step_efforts:=[0.01, 0.25, 0.50]" \
  -p stop_duration_s:=2.5 \
  -p step_duration_s:=6.0 \
  -p publish_rate_hz:=50.0 \
  -p max_speed_mps:="$GUARD_CHINH_THUC"
```

`0.6 m/s` là guard chính thức đã chốt cho các lượt fit Gate 2. Nếu filtered
vượt guard này, rolling source phải dừng/abort; không tự tăng guard trong lúc
đang chạy một lượt test.

Với `stop_duration_s:=2.5` và `step_duration_s:=6.0`, một sequence gồm ba
mức kéo dài khoảng `28s` (stop `2,5s` + step `6s` cho mỗi mức và final stop
`2,5s`). Recorder nên chạy
`60s` để có thời gian chuẩn bị, chạy sequence và thu phần dừng cuối.

Rolling source và actuation manager hiện phát lệnh ở `50 Hz`. Giữ
`esc_rise_rate_us_per_s=50` để giới hạn gia tốc PWM; tăng tần số chỉ làm các
bước PWM nhỏ và đều hơn, không làm tăng tốc độ đích.

Với map ngắn, chạy nhiều lượt ngắn, mỗi lượt stop/disarm và đưa xe về vị trí
xuất phát. Các CSV phải được aggregate vào cùng evidence trước khi fit. Không
coi một session chỉ có một PWM là đủ model-fit.

Khi CSV đã có đủ dữ liệu:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/latest

bash scripts/fit_longitudinal_model.sh "$SESSION" 6.5
cat "$SESSION/plots/longitudinal_model/report.md"
```

Nếu cần fit một session cũ cụ thể, thay `evidence/latest` bằng đường dẫn thật
được in ra từ `new_session.sh`, không đặt đường dẫn trong dấu `< >`.

Chỉ coi Gate 2 PASS khi có tối thiểu 50 armed transitions, đủ đa dạng PWM và
held-out velocity RMSE `<= 0.04 m/s`. Không đặt `model_calibrated=true` bằng tay
trước khi report đạt gate.

## 11. Lệnh khẩn cấp

Disarm:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: false}"
```

E-stop:

```bash
ros2 service call /actuation/estop \
  std_srvs/srv/Trigger "{}"
```

Nếu xe có dấu hiệu bất thường: E-stop trước, disarm sau, giữ nguyên evidence
để phân tích. Không retry trong cùng session sau `ABORTED_*`.
