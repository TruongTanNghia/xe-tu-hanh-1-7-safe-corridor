# Runbook test phần cứng

## Trạng thái gate hiện tại — 2026-07-16

- Full-loop shadow + micro-ROS/actuation DISARMED đã PASS:
  `evidence/20260711-172406_full_loop_actuation_disarmed`.
- `steering_calibrated=true` và `esc_calibrated=true` trong config actuation.
- `control_rate_hz=50.0`; rolling source fit dùng `publish_rate_hz=50.0`.
- NMPC hardware config mặc định dùng `shadow_mode=false` cho các phiên test có
  kiểm soát; đã merge candidate với `model_calibrated=true` sau review.
- Gate 1 đã xác nhận runtime actuation DISARMED và calibration flags.
- Gate 2 v2 model-fit đã PASS tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
  184 armed transitions, đủ ba PWM, held-out RMSE `0,007438 m/s`, bag `59,4s`.
- Runtime trong evidence xác nhận actuation/control khoảng `50 Hz`, filtered
  khoảng `6,17 Hz`, vận tốc armed tối đa `0,381 m/s` dưới guard `0,6 m/s`.
- Review lưu ý process thực tế dùng `stop_duration_s:=1.0`, dù tên scenario ghi
  `stop2p5`; không dùng session này để kết luận riêng về stop `2,5s`.
- Đã xác định và sửa `sensor_timeout=0,3→0,15 s` của EKF; build/test
  `rc_localization` `6/6` PASS.
- Gate 3 active pre-arm đã PASS tại
  `evidence/20260715-095303_gate3_default_model_prearm_20260715`; held active
  cũng đã PASS theo trạng thái dự án, còn Gate 5 rolling slow chưa PASS.
- Gate 5 test 1, 2 và 3 full map đều chưa đạt ba vòng liên tiếp. Test 3 với
  exposure `4000 us` active khoảng `21,5 s`, vượt đoạn chuyển làn đơn nhưng
  safe-stop ở vùng cua gắt. Report:
  `evidence/20260716-115602_gate5_full_map_rolling_exposure4000_20260716/report.md`;
  video: `evidence/test3_gate5.mp4`.
- Exposure `4000 us` cho kết quả ban đầu tốt hơn `8000 us`, nhưng chưa đủ để
  kết luận lỗi chỉ do camera. Việc tiếp theo là phân tích/nâng cấp perception
  theo ánh sáng và cua; sau đó mới tách riêng khả năng lỗi NMPC hoặc tầng khác.

Lệnh chạy Gate 5 được tách riêng theo terminal tại
[`gate5_manual_test.md`](gate5_manual_test.md).

Lượt Gate 5 cuối `evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716`
cũng chưa PASS: armed khoảng `30,8 s`, có `solver_failed`, `lane_invalid` và
safe-stop. Bước kế tiếp là diagnostic image processing + servo-only; xem
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).

## Nguyên tắc trước khi mở terminal

1. Xác nhận ai đang chạy process trên Pi; không dừng process không do mình tạo.
2. G0–G1: ngắt nguồn động lực. Gate 2 v2: xe trên sàn, tốc độ thấp, E-stop và
   người vận hành sẵn sàng. Gate held: xe bị giữ chắc.
3. Mỗi lần test tạo một evidence session; FAIL thì quay lại gate gần nhất.
4. Không sửa YAML trong lúc session đang record.

## Terminal 1 — preflight và bringup

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/bootstrap_acados.sh --verify-only
bash scripts/preflight.sh --hardware
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
ros2 launch rc_bringup shadow_full.launch.py
```

Profile này không chạy micro-ROS/actuation và launch NMPC sẽ ép
  `shadow_mode=true`, dù hardware config mặc định dùng active cho các phiên
  hardware test có kiểm soát.

## Terminal 2 — quan sát từ SSH

Lane:

```bash
bash scripts/observe_lane_web.sh
```

Forward port `8080`. LiDAR:

```bash
bash scripts/run_lidar_web_viewer.sh
```

Forward port `8090`. Web viewer LiDAR là đường ưu tiên khi DDS qua Wi-Fi fail;
không cần ROS 2 trên laptop.

## Terminal 3 — ghi dữ liệu

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario morning_shadow)
bash scripts/evidence/observe.sh --session "$SESSION" --duration 10
bash scripts/evidence/record.sh --session "$SESSION" --duration 60
```

## Terminal 4 — đánh giá và đóng session

```bash
bash scripts/evidence/evaluate.sh --session "$SESSION" --mode lidar
bash scripts/evidence/evaluate.sh --session "$SESSION" \
  --mode fusion --bag "$SESSION/rosbag/latest" --expected-speed 0.0
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Nếu một evaluator FAIL, đóng session với `--result FAIL`; không đổi thành PASS
bằng tay.

## Thứ tự gate

| Gate | Nguồn động lực | Việc làm |
|---|---|---|
| Gate 0 | OFF | build/test, lane, LiDAR, fusion đứng yên, NMPC shadow |
| Gate 1 | DISARMED | actuation config, heartbeat, stop state, publisher gate |
| Gate 2 v2 | ON có kiểm soát | **PASS** — LiDAR bật, rolling model-fit, guard `0,6 m/s` |
| Gate 3 | DISARMED | active NMPC pre-arm và fault-to-stop |
| Gate 4 | ON, xe bị giữ | active NMPC held + E-stop |
| Gate 5-pre | Motor/ESC OFF | đẩy xe bằng tay, chẩn đoán image processing + servo |
| Gate 5 | ON có kiểm soát | rolling slow tự hành |

Nếu một gate đã có evidence PASS trong `project_state.md`, không cần làm lại
trừ khi source/config/runtime đã đổi. Với trạng thái hiện tại, Gate 1, Gate 2
model-fit, Gate 3 active pre-arm và Gate 4 held active đã PASS. Gate 5 vẫn chưa
PASS; xem [`gate5_manual_test.md`](gate5_manual_test.md) trước lượt chạy kế tiếp.
