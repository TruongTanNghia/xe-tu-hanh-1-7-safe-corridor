# Bước 3 — NMPC shadow và active có kiểm soát

## Trạng thái hiện tại — 2026-07-16

Hardware config mặc định cho các phiên test có kiểm soát:

```yaml
shadow_mode: false
model_calibrated: true
```

Full-loop shadow + actuation DISARMED đã PASS tại
`evidence/20260711-172406_full_loop_actuation_disarmed`. Active NMPC chỉ được
chạy bằng override có chủ đích cho gate pre-arm/held/rolling và phải record
evidence. Candidate đã được review/merge sau khi đạt gate model-fit.

Gate 2 v2 model-fit đã PASS tại
`evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`:
184 armed transitions, đủ ba mức PWM `1580/1585/1590 us`, actuation/control
khoảng `50 Hz`, filtered khoảng `6,17 Hz`, held-out RMSE `0,007438 m/s`.
Candidate đã được review/merge vào default; Gate 3 active pre-arm đã PASS tại
`evidence/20260715-095303_gate3_default_model_prearm_20260715`. `cr3=20.0`
vẫn là bound-edge parameter cần theo dõi khi chạy thực.

Gate 5 test 1–3 ngày 2026-07-16 chưa PASS. Test 3 với exposure `4000 us` có
`1100 ACTIVE_VALID`, `18 lane_invalid`, `7 cte_outside_safe_bound` và
`2 solver_failed`, sau đó safe-stop; tốc độ filtered tối đa `0,504 m/s`.
Lượt cuối sau đó vẫn chưa đủ ba vòng, armed khoảng `30,8 s`, có
`16 solver_failed:4` trong khoảng armed và safe-stop. Không chuyển sang
autonomous rerun ngay; trước hết chạy diagnostic perception + servo-only theo
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).
lane invalid xảy ra cùng các đoạn cua gắt, chưa thể kết luận NMPC là nguyên
nhân gốc trước khi perception/reference được kiểm tra ổn định hơn. Không nới
constraint hoặc tắt fault gate chỉ để đạt ba vòng.

## 1. Chuẩn bị

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/build.sh race_cars rc_bringup
bash scripts/test.sh race_cars
```

NMPC target `0,6 m/s`, chạy 20 Hz, `Tf=1,3 s`, `N=10`. Các thông số thuật
toán được đồng bộ trực tiếp từ source `autonmous_car_research/src/race_cars`
commit `0b0723b`. `shadow_mode=false` là mặc định của hardware config để
chuẩn bị Gate 4. Profile `nmpc_shadow.launch.py` và `shadow_full.launch.py`
vẫn ép `shadow_mode=true`; active chỉ được dùng sau khi record evidence,
pre-arm gate sạch và E-stop sẵn sàng.

Các thông số thuật toán giữ theo source research, nhưng constraint vật lý dùng
xe thật: duty `[0,1]` vì không có reverse, góc lái `±0,524 rad` theo calibration
`±30°`, gia tốc dọc/ngang `±2 m/s²`. Riêng CTE dùng `±0,45 m` theo phần đường
chuyển làn kép thực tế để controller không dừng giữa đoạn. Lớp `rc_actuation`
tiếp tục đổi effort/góc lái sang PWM và clamp lần cuối.

Benchmark arm64 ngày 2026-06-29: 500 solve, 0 failure, P99 `0,387 ms`, max
`0,478 ms`. Smoke với input hợp lệ xác nhận dự đoán chỉ nằm ở
`/nmpc_cmd_shadow`; typed command vẫn `valid=false` và `/nmpc_cmd=[0,0]`.
Evidence: `evidence/20260629-075216_nmpc_shadow_software_benchmark`.

## 2. Lệnh chạy

Chạy riêng NMPC khi sensor/fusion đã hoạt động:

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh
ros2 launch race_cars nmpc_shadow.launch.py
```

Hoặc chạy toàn pipeline an toàn:

```bash
ros2 launch rc_bringup shadow_full.launch.py
```

Chạy sensors không kèm NMPC để chuẩn bị active pre-arm:

```bash
ros2 launch rc_bringup shadow_full.launch.py start_nmpc:=false
```

Chạy NMPC active bằng override cho gate có kiểm soát:

```bash
ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/install/race_cars/share/race_cars/config/nmpc_hardware.yaml \
  -p shadow_mode:=false
```

Chỉ arm nếu `/nmpc/status=ACTIVE_VALID`, `/control/nmpc.valid=true`, publisher
gate sạch, E-stop sẵn sàng và đang record evidence.

## 3. Quan sát

```bash
ros2 topic echo /lane/reference --once
ros2 topic echo /odometry/filtered --field twist.twist.linear.x
ros2 topic hz /control/nmpc
ros2 topic echo /nmpc/status
ros2 topic echo /nmpc/solve_time_ms
ros2 topic echo /nmpc_cmd
ros2 topic echo /nmpc_cmd_shadow
```

Trong shadow, dự đoán nằm ở `/nmpc_cmd_shadow`; `/nmpc_cmd` phải luôn `[0,0]`
và `ControlCommand.valid=false`, reason `SHADOW_MODE`.

## 4. Xuất dữ liệu

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario nmpc_shadow)
bash scripts/evidence/observe.sh --session "$SESSION" --duration 10
bash scripts/evidence/record.sh --session "$SESSION" --duration 60 -- \
  /lane/reference /odometry/filtered /control/nmpc \
  /nmpc_cmd /nmpc_cmd_shadow /nmpc/status /nmpc/solve_time_ms
```

Benchmark solver:

```bash
ros2 run race_cars benchmark_solver --iterations 500 \
  --output "$SESSION/logs/nmpc_benchmark.json"
```

## 5. Đánh giá

PASS shadow khi:

- 500 lần solve không failure và P99 `< 50 ms`;
- mất lane, lane stale, odom stale, NaN hoặc CTE ngoài ±0,45 m đều phát stop;
- `/nmpc_cmd` không có throttle;
- không có publisher `/mcu/actuator_cmd` từ NMPC.

Chưa được active cho tới khi step-response vật lý fit model và held-out velocity
RMSE `≤ 0,04 m/s`. Các hệ số hiện tại chỉ là provisional từ simulation.

### Gate 2 v2 — rolling calibration trên sàn (đã PASS)

Gate 2 wheels-up trước đây không xác nhận được vận tốc xe: LiDAR đã tắt nên
recorder thu 0 mẫu và fitter báo `need at least 50 valid armed transitions, got
0`. Không dùng `run_esc_step_calibration.sh` cho xe đang đặt trên sàn; source
đó chỉ dành cho wheels-up.

Gate 2 v2 dùng LiDAR bật và xe đặt trên sàn. Người vận hành chịu trách nhiệm
E-stop, hướng chạy và kiểm soát thủ công; source calibration riêng không chạy
NMPC, không tự arm, bắt đầu ở `READY_STOP`, kiểm tra odometry fresh và guard
`max_speed_mps=0,6 m/s`. Runtime fit đã dùng actuation/control `50 Hz`,
filtered khoảng `6,17 Hz`, ba mức PWM `1580/1585/1590 us` và fitter PASS.
Process snapshot của evidence PASS thực tế dùng `stop_duration_s:=1.0` dù tên
scenario ghi `stop2p5`; hành vi stop `2,5s` chưa được xác nhận riêng.

### Session rolling chẩn đoán lịch sử — không dùng để đánh giá PASS hiện tại

Session `evidence/20260713-171843_longitudinal_fit_rolling_v2_60s` đã hoàn tất
ghi dữ liệu tự nhiên, không bị mất CSV:

- rosbag dài khoảng `59,48 s`, longitudinal CSV có `221` mẫu;
- có `37` mẫu `armed=1`, trong đó `6` mẫu ở PWM `1580`; armed flag của MCU
  hoạt động đúng. `armed` là bit trạng thái MCU, không phải giá trị PWM;
- sequence dừng an toàn khi odometry báo `vx ≈ 0,400 m/s`, vượt giới hạn
  `0,35 m/s`, nên chưa đi tới các mức PWM `1585–1600`;
- chỉ có `15` transition hợp lệ, chưa đạt tối thiểu `50`, và chưa tạo được
  `plots/longitudinal_model/report.md`;
- `/scan` và RF2O khoảng `6,4 Hz`, nhưng `/odometry/filtered` trong session
  khoảng `3,65 Hz`. Fitter loại các cặp mẫu có `dt > 0,25 s`, nên đây là
  blocker dữ liệu cần xử lý.

Kết luận lịch sử: session này là evidence chẩn đoán, không phải evidence PASS.
Các kết quả này không thay thế evidence Gate 2 PASS ngày 2026-07-14.
`TEMP_GUARD=1,0 m/s` đã được operator chốt chỉ cho probe này; không dùng nó làm
guard chính thức và không bật `model_calibrated=true` trước khi có fit report
đạt gate.

Phân tích chi tiết cho thấy tại PWM `1580`, RF2O raw đạt khoảng `0,474 m/s`
và tốc độ tính từ delta pose cũng khoảng `0,46–0,50 m/s`; conditioner và EKF
chỉ làm giá trị lần lượt còn khoảng `0,403` và `0,443 m/s`. Do đó đây không
phải chỉ là spike do EKF. Giới hạn `0,35 m/s` đang hoạt động đúng; cần xác nhận
mức PWM/ESC phù hợp khi xe trên sàn hoặc thử một mức PWM sàn thấp hơn, không
được dùng `TEMP_GUARD=1,0 m/s` như guard chính thức.

Cũng từ session lịch sử, cadence thấp có nguyên nhân cấu hình:
`sensor_timeout=0,3 s` trong EKF. Đã đổi
thành `0,15 s` để cho phép filtered vượt 6 Hz theo nguồn RF2O khoảng 6,4 Hz.
Build và unit test `rc_localization` `6/6` PASS; phải restart runtime thật và
đo lại trước khi thu session mới.

Tạo evidence và quan sát trước khi arm:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

SESSION=$(bash scripts/evidence/new_session.sh \
  --scenario longitudinal_fit_rolling_v2)
bash scripts/evidence/observe.sh --session "$SESSION" --duration 10
```

Giữ nguyên `SESSION` ở các terminal kế tiếp. Mở recorder rosbag và CSV ở hai
terminal riêng, trước khi chạy source step:

```bash
SESSION=/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/<timestamp>_longitudinal_fit_rolling_v2
bash scripts/evidence/record.sh --session "$SESSION" --duration 120
```

Terminal khác:

```bash
bash scripts/record_longitudinal_model.sh "$SESSION" 120.0
```

Phải dùng `120.0`, không dùng `120`: ROS 2 khai báo `duration_sec` là kiểu
`DOUBLE`, truyền số nguyên sẽ gây `InvalidParameterTypeException` trước khi
recorder chạy.

Terminal calibration khác:

```bash
bash scripts/run_esc_rolling_calibration.sh --rolling-slow-confirmed
```

Kiểm tra source đang ở stop và không có publisher điều khiển khác:

```bash
ros2 topic echo /calibration/rolling_step_status --once
ros2 topic info -v /control/nmpc
```

Chỉ khi status là `READY_STOP`, publisher duy nhất của `/control/nmpc` là
`rolling_step_source`, recorders đang chạy và E-stop sẵn sàng, mới arm rồi bắt
đầu sequence:

```bash
ros2 service call /actuation/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /calibration/rolling_start std_srvs/srv/Trigger "{}"
```

Theo dõi trong lúc chạy:

```bash
ros2 topic echo /calibration/rolling_step_status
ros2 topic echo /odometry/filtered --field twist.twist.linear.x
ros2 topic echo /actuation/status
```

Nếu source báo `ABORTED_*`, xe vượt giới hạn tốc độ hoặc odometry stale, không
retry sequence trong cùng session: disarm/E-stop, giữ lại log và đánh giá
session là FAIL.

Nếu sequence abort vì tốc độ, không tăng `max_speed_mps` ngay. Giữ giới hạn
`0,35 m/s`, đối chiếu `/odometry/filtered` với `/odometry/rf2o_raw` và xác nhận
xe có thật sự vượt tốc độ hay không. Nếu odometry đúng, cần giảm bước/đầu vào
rolling; nếu odometry sai hoặc thưa, cần sửa pipeline đo trước khi thu model.

Trong trường hợp đã xác nhận RF2O raw và delta pose cùng vượt `0,35 m/s` như
session hiện tại, không coi đó là lỗi filtered. Khi đó phải hiệu chỉnh lại
deadband/forward-start của ESC hoặc tìm mức PWM sàn thấp hơn thông qua một test
riêng có giới hạn; không sửa guard để cho phép xe chạy nhanh hơn.

### Lộ trình Gate 2 trên map ngắn — đã hoàn tất

`1595 us` được dùng như một probe riêng, không phải mức PWM đưa vào model-fit:

1. Record evidence trước khi arm.
2. Chạy probe `1595 us` trong thời gian rất ngắn, với guard tạm thời bảo thủ,
   E-stop sẵn sàng và operator kiểm soát hướng xe.
3. Dựa trên vận tốc đo được và biên an toàn, review/chốt guard mới. Nếu probe
   vượt giới hạn tạm thời thì abort và không dùng kết quả đó để hợp thức hóa
   việc tăng guard.
4. Chạy model-fit bằng ba mức `1580/1585/1590 us`, chia thành nhiều lượt ngắn;
   giữa các lượt stop/disarm và đưa xe về vị trí xuất phát.
5. Aggregate các lượt trong cùng evidence, đạt tối thiểu 50 transition và
   held-out RMSE `<= 0,04 m/s`; điều kiện này đã đạt trong evidence chính thức.

Không yêu cầu bánh xe chạy liên tục 60 giây. Thời lượng recorder có thể dài hơn
thời gian xe chuyển động để giữ đủ bag/log, nhưng mọi đoạn chuyển động phải
nằm trong khu vực test và dưới guard đã được review.

Sau khoảng 43 s, hoặc ngay khi có bất thường, disarm rõ ràng. Các lệnh dưới đây
là quy trình kết thúc đã dùng:

```bash
ros2 service call /actuation/arm std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
```

Dừng source bằng Ctrl+C đúng terminal do mình mở; chờ cả hai recorder kết thúc.
Longitudinal CSV phải có ít nhất 50 armed transitions hợp lệ. Sau đó fit:

```bash
bash scripts/fit_longitudinal_model.sh "$SESSION" 6.5
```

Fitter chia train/held-out theo thời gian và tạo
`plots/longitudinal_model/nmpc_model_candidate.yaml`; nó không tự sửa config.
Report chính thức đã đạt held-out velocity RMSE `0,007438 m/s`. Không bật
`model_calibrated=true` chỉ vì fitter tạo được YAML; candidate phải được review
và merge có chủ đích trước Gate 3.

## 6. Lưu bằng chứng

```bash
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Lưu cùng report fusion và model-fit. Benchmark nhanh không thay thế test
freshness/fault injection.
