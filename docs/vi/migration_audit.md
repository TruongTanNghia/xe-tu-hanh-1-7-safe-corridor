# Kiểm toán migration

## Cập nhật trạng thái — 2026-07-16

Các kết luận migration bên dưới vẫn là lịch sử. Trạng thái runtime hiện tại đã
tiến thêm:

- Gate 2 v2 model-fit: **PASS** tại
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`;
- held-out RMSE: `0,007438 m/s`, `184` armed transitions, đủ PWM
  `1580/1585/1590 us`;
- actuation/control khoảng `50 Hz`, filtered khoảng `6,17 Hz`, guard
  `0,6 m/s`;
- candidate YAML đã được review/merge với `model.cr3=20.0`; config mặc định
  hiện là `model_calibrated=true`, `shadow_mode=false` cho phiên hardware test
  có kiểm soát; Gate 3 active pre-arm và Gate 4 held active đã PASS;
- `/bno055/imu` vẫn chưa có publisher runtime, nên chưa tuyên bố full
  BNO055/EKF fusion.
- Gate 5 test 1–3 full map ngày 2026-07-16 đều chưa PASS. Test 3 dùng exposure
  `4000 us`, vượt chuyển làn đơn nhưng safe-stop trong cua gắt; xem
  `evidence/test3_gate5.mp4` và report test 3.
- Lượt cuối cùng cũng chưa PASS; trạng thái hiện tại chuyển sang diagnostic
  image processing + servo-only trước khi điều tra NMPC tiếp. Runtime chưa có
  `servo_only`; motor/ESC phải vô hiệu hóa vật lý và ESC phải giữ `1550`.

## Mốc nguồn

| Repository | Branch | Commit |
|---|---|---|
| `autonmous_car_research` | `test3-nmpc-validation-cleanup` | `0b0723bb606d24336ed46dacc4a1d716c65886f5` |
| `autonomous-car-hardware-platform` | `main` | `865612216760b85511c2c4ec324e929840bc0e62` |
| V2 worktree | `hardware-platform-v2` | base `865612216760b85511c2c4ec324e929840bc0e62` |

## Ma trận quyết định

| Thành phần | Nguồn | Quyết định | Lý do/test |
|---|---|---|---|
| `image_processing` | research | Giữ thuật toán | Build + live viewer; người vận hành chấp nhận để port |
| bicycle/NMPC model | research | Port có sửa | Packaging chuẩn, typed input, freshness, shadow, benchmark |
| `rc_perception` | research | Loại | Không phải pipeline xử lý ảnh mục tiêu |
| Gazebo/description | research | Không đưa vào runtime Pi | Chỉ là simulation asset |
| A1M8/RF2O/BNO055 | hardware cũ | Giữ ý tưởng, thay dependency bằng commit pin | Vendor build độc lập |
| EKF cũ | hardware cũ | Không dùng nguyên trạng | Fuse `ax, ay`, timeout 0,1 s, topic remap không canonical |
| PWM scripts rời | hardware cũ | Loại khỏi runtime | Nhiều publisher, không typed/freshness/arm gate |
| STM32 heartbeat cũ | archive | Chỉ dùng skeleton HAL/micro-ROS | Hằng PWM lệch, không arm, không reconnect, path tuyệt đối |

## Dependency đã khóa

| Dependency | Commit |
|---|---|
| `vision_opencv` | `9800f67cea477c44cfb64e349854bcb6a09dc9ce` |
| `bno055` | `45e1ff16936101711260c9fda63fbad99376ce3b` |
| `rf2o_laser_odometry` | `b38c68e46387b98845ecbfeb6660292f967a00d3` |
| `sllidar_ros2` | `34300099fadfc772965962dec837bf436706188f` |
| `micro-ROS-Agent` | `4f363a79ae96aed39dcc5269f53f73f6a70d3a2c` |
| `micro_ros_msgs` | `100bf269e78da3fe0a58f6531ebe47d6991bd9ab` |
| `acados` | `8e1a6f856e063c423de583e03c691e3b2b7fc0a0` |

Sáu dependency ROS được khôi phục bằng `scripts/bootstrap.sh`. acados nằm ngoài
colcon workspace và được kiểm tra/build bằng `scripts/bootstrap_acados.sh`;
không phụ thuộc một `PYTHONPATH` chép tay.

Acceptance test mặc định chỉ chạy tám package do V2 sở hữu. Audit
`scripts/test.sh --include-vendor` ngày 2026-06-29 có một failure lint copyright
trong `micro_ros_msgs/CONTRIBUTING.md` (`<unknown>`); build và package chức năng
vẫn PASS. Không sửa source vendor chỉ để che lỗi lint upstream.

## Thay đổi có chủ đích

### Fusion

EKF cũ fuse gia tốc BNO055 cùng `vx`; cách này biến bias/rung 100 Hz thành drift
vận tốc. Tần số 100 Hz tự nó không sai. V2 chỉ fuse RF2O `vx` và gyro `z`, gán
covariance khác 0, lọc outlier/median/EMA và đánh giá bằng CSV.

LiDAR A1M8 thật đã PASS đường web live và rosbag replay ngày 2026-06-29. Điều
này chỉ xác nhận sensor/quan sát; chưa xác nhận RF2O/EKF khi xe chuyển động.

Runtime RF2O đã được đổi tên node chính thành `/v2_rf2o_laser_odometry` trong
constructor và bỏ `name=` global remap ở launch. Cách này giữ nguyên executable,
topic và thuật toán, đồng thời loại duplicate node nội bộ; `/odometry/rf2o_raw`
hiện có đúng một publisher.

Ngày 2026-07-13, `/scan` và RF2O quan sát khoảng 6,4 Hz. `/bno055/imu` đang có
0 publisher runtime, nên trạng thái hiện tại là LiDAR/RF2O/EKF với gyro IMU
chưa available; không ghi là full BNO/EKF fusion.

Driver BNO055 upstream đọc dữ liệu đúng nhưng entry point của commit đã khóa
shutdown `rclpy` hai lần khi Ctrl+C, gây traceback giả lúc teardown.
`rc_localization/bno055_safe_node` chỉ thay lifecycle/timer ownership, tiếp tục
dùng nguyên transport, register handling và publisher upstream. Smoke phần
cứng sau sửa kết thúc sạch 5/5 process.

### NMPC

Source research dùng layout Python không chuẩn và generated acados trong source.
V2 đóng package chuẩn, cache solver ngoài repo, thêm `LaneReference`,
`ControlCommand`, CTE ±0,45 m theo source research, fault-to-stop và cấm active
khi model chưa fit.

### Firmware

Archive gốc được giữ nguyên. Firmware mới dùng stop `1550`, forward
`1580..1600`, servo `1150..1850`, timeout 250 ms, arm heartbeat, reconnect,
applied-state và application-level stop timeout. Hardware IWDG đang tắt trong
bring-up, nên không được tuyên bố có independent watchdog. Source verification PASS; target
build/link/flash còn chờ STM32CubeIDE.

## Những gì chưa được phép tuyên bố

- Extrinsic LiDAR/IMU đã có status `measured_ccw_verified` và evidence lịch sử;
  BNO055 live publisher hiện chưa healthy nên chưa ký lại full IMU/EKF runtime.
- Fusion đứng yên 59,48 giây đã PASS; gate quay CCW đã có evidence lịch sử.
  Runtime BNO hiện chưa publish. Gate 2 rolling dọc đã có evidence chẩn đoán
  tại `evidence/20260713-171843_longitudinal_fit_rolling_v2_60s`, nhưng chưa
  PASS: sequence abort ở `0,400 m/s`, chỉ có 15 transition hợp lệ và filtered
  odometry khoảng 3,65 Hz trong khi RF2O khoảng 6,4 Hz. Phân tích raw/pose
  xác nhận tốc độ cao đã xuất hiện từ RF2O; đồng thời đã sửa EKF
  `sensor_timeout=0,3→0,15 s`, build/test `6/6` PASS. Runtime thật và PWM/ESC
  sàn vẫn cần xác nhận trước khi đóng Gate 2. Kế hoạch là probe
  `1595 us` riêng để chốt guard có margin, sau đó fit bằng `1580/1585/1590 us`
  qua các lượt ngắn được aggregate.
- Model dọc NMPC đã fit từ xe thật và Gate 2 model-fit đã PASS; `cr3=20.0`
  vẫn là tham số chạm bound cần theo dõi khi chạy thực.
- Firmware mới chưa build target và chưa flash.
- Xe chưa hoàn tất ba vòng liên tiếp ở Gate 5 với tốc độ mục tiêu `0,6 m/s`.

Các mục trên phải chuyển trạng thái bằng evidence, không bằng sửa chữ trong
tài liệu.
