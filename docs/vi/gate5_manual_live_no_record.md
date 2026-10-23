# Gate 5 exploratory — full-loop live, không ghi dữ liệu

Tài liệu này dùng cho một lượt quan sát thủ công trên full map bằng web panel.
Lượt này **không tạo evidence, không ghi rosbag/CSV và không đủ điều kiện kết
luận Gate 5 PASS**. Mục tiêu là để operator quan sát bốn panel trong lúc xe
chạy và phát hiện nhanh hiện tượng bất thường.

Nếu cần kết luận Gate 5 hoặc phân tích sau phiên, dùng
[`gate5_manual_test.md`](gate5_manual_test.md) với rosbag đầy đủ.

## Build lại sau khi chỉnh `race_cars`

Nếu bạn của bạn thay đổi code NMPC hoặc
`ros2_ws/src/race_cars/config/nmpc_hardware.yaml`, chạy build/test một lần
trước khi mở pipeline runtime:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source scripts/acados_env.sh

bash scripts/build.sh race_cars
bash scripts/test.sh race_cars
```

Kỳ vọng: build thành công và unit test `race_cars` PASS. Nếu có lỗi, dừng tại
đây; không chạy xe với bản build chưa được xác nhận.

Sau build, mọi terminal chạy node phải source lại workspace:

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh
```

Node NMPC đang chạy không tự đọc lại config. Phải Ctrl+C node NMPC cũ do chính
phiên này khởi động rồi chạy lại:

```bash
ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/install/race_cars/share/race_cars/config/nmpc_hardware.yaml \
  -p shadow_mode:=false
```

Không build hoặc restart bằng cách dừng process của session khác. Nếu chỉ thay
đổi `race_cars`, lệnh `bash scripts/build.sh race_cars` là đủ; nếu đồng thời
thay đổi package khác, thêm package đó vào lệnh build theo workflow chính.

## Điều kiện an toàn

- Xe ở đúng vị trí xuất phát, khu vực trống, E-stop trong tầm tay và có người
  giám sát.
- Không có người/vật cản trong vùng xe chạy.
- Chỉ chạy một pipeline; không dùng `pkill`, `killall` hoặc `ros2 daemon stop`.
- Chỉ Ctrl+C các process do chính lượt này khởi động.
- Không arm nếu pre-arm chưa đạt.
- Khi panel mất line, CTE/heading nhảy mạnh, solver lỗi, odometry stale hoặc
  actuation không về safe state: gọi disarm ngay.

## Terminal 0 — setup và kiểm tra một lần

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

SERIAL_DEVICE=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
test -e "$SERIAL_DEVICE" && echo "Serial OK" || echo "Serial NOT FOUND"
readlink -f "$SERIAL_DEVICE"
ps -eo pid,ppid,stat,cmd | rg \
  'ros2 launch hardware_camera|run_fusion.sh|lane_reference_adapter|racecar_nmpc_node|run_micro_ros_actuation|web_viewer' \
  | rg -v 'rg ' || true
```

Kỳ vọng: serial tồn tại, thường trỏ tới `/dev/ttyACM0`, và không có pipeline
cũ đang chạy. Nếu serial không tồn tại hoặc có process không rõ chủ sở hữu,
dừng tại đây.

## Terminal 1 — camera và image processing, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

ros2 launch hardware_camera lane_detection.launch.py \
  manual_controls:=true \
  exposure_time_us:=4000 \
  analogue_gain:=1.0 \
  boundary_corridor_enabled:=true \
  boundary_margin_px:=4 \
  boundary_recovery_enabled:=true \
  boundary_recovery_max_age_ms:=150 \
  boundary_recovery_min_observations:=3 \
  boundary_search_left_ratio:=0.0 \
  boundary_search_right_ratio:=1.0 \
  scan_top_ratio:=0.35 \
  scan_bot_ratio:=0.75
```

Kỳ vọng: camera bắt đầu với `ExposureTime=4000 us`, gain `1.0`; log có các
giá trị `bands`, `CTE`, `heading`, `kappa`. Đây là terminal chạy lâu dài.

### Cách kiểm tra `boundary_corridor`

Với `boundary_corridor_enabled:=true`, khi processor fit được cả hai biên
vàng, toàn bộ vùng nằm giữa hai biên được đưa vào scan mask. Vì vậy phần mặt
đường bị chá sáng một phía vẫn được xem là corridor liên tục thay vì bị mất
khỏi scanband do ngưỡng độ sáng Otsu. `boundary_margin_px:=4` chừa một khoảng
an toàn phía trong hai biên để tránh đưa chính vạch vàng vào vùng đường.

Trong panel **Debug (scan bands)**, kiểm tra các điểm sau:

- scanband không bị đứt tại phần mặt đường chá sáng;
- số `bands` hợp lệ ổn định, không tụt đột ngột khi xe vào vùng sáng;
- `CTE`, `heading` và `kappa` thay đổi liên tục, không nhảy mạnh chỉ vì đổi
  ánh sáng;
- hai biên vàng trong panel **Poly Fitting** vẫn được fit đúng trước khi
  corridor được sử dụng.

Nếu một scanband đột nhiên kéo dài gần toàn chiều rộng ảnh, đó là dấu hiệu
corridor không hợp lệ hoặc đang fallback. Không xem đó là kết quả đúng: dừng
lượt test và ghi nhận frame đó để kiểm tra. Processor hiện đã bỏ qua hàng có
polynomial vượt ngoài ảnh thay vì clamp nó vào hai mép ảnh.

Giải pháp này chỉ kích hoạt khi cả hai biên vàng được nhận diện đủ tin cậy.
Nếu biên vàng cũng bị mất do ánh sáng, hệ thống fallback về Otsu cũ và không
được xem là đã khắc phục hoàn toàn vấn đề.

Khi một biên chỉ mất một đoạn, processor có thể fit riêng các đoạn còn nhìn
thấy của từng biên. Nếu chỉ còn một biên trong frame hiện tại, processor dùng
độ rộng corridor của frame hợp lệ trước đó trong tối đa `150 ms` để phục hồi
tạm thời. Đây không phải là giữ lane vô thời hạn: hết thời gian này hoặc hình
học corridor không hợp lệ thì phải quay về `line_invalid`/safe-stop.

### A/B với pipeline cũ

Không đổi parameter trong lúc xe đang chạy. Nếu cần so sánh, dừng pipeline
hiện tại bằng Ctrl+C, sau đó chạy một lượt riêng với:

```bash
ros2 launch hardware_camera lane_detection.launch.py \
  manual_controls:=true \
  exposure_time_us:=4000 \
  analogue_gain:=1.0 \
  boundary_corridor_enabled:=false
```

Lượt mặc định dùng `boundary_corridor_enabled:=true` ở trên; lượt `false` chỉ
dùng làm baseline Otsu. Không kết luận Gate 5 PASS từ lượt quan sát live này.

## Terminal 2 — LiDAR/RF2O/EKF, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

bash scripts/run_fusion.sh start_imu:=false
```

`start_imu:=false` là chủ ý vì runtime hiện không có publisher BNO055. Kỳ vọng:
LiDAR, RF2O và `/odometry/filtered` có dữ liệu mới.

## Terminal 3 — lane reference adapter, chạy lâu dài

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

Kỳ vọng: NMPC báo active, model calibrated và không có lỗi solver liên tục.

## Terminal 5 — micro-ROS actuation, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

bash scripts/run_micro_ros_actuation.sh \
  serial_device:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02
```

Kỳ vọng ban đầu: `Actuation starts DISARMED`, MCU heartbeat có mặt,
`servo=1500`, `esc=1550`. Đây là terminal chạy lâu dài.

## Terminal 6 — web panel, chạy lâu dài

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/observe_lane_web.sh
```

Forward port `8080` trong VS Code/SSH rồi mở địa chỉ forwarded. Mặc định web
viewer chỉ mở **Trang Tracking / Output** để giảm tải Raspberry Pi; không được
coi việc tắt trang Segmentation là tắt image processing trong ROS.

**Trang Tracking / Output**:

1. **Boundary Geometry** — boundary quan sát, fit và corridor cuối;
2. **Scan Bands** — scanband và centroid;
3. **Poly Fitting** — đường fit và điểm dữ liệu;
4. **Metrics / Debug Mode** — mode `FULL_CORRIDOR`, `RECOVER_LEFT`,
   `RECOVER_RIGHT` hoặc `OTSU_FALLBACK`, cùng status, confidence, CTE, heading,
   kappa.

Nếu cần bật lại trang Segmentation cho một phiên debug riêng, dùng:

```bash
LANE_VIEWER_SHOW_SEGMENTATION=true \
LANE_VIEWER_MAX_FPS=3 \
LANE_VIEWER_JPEG_QUALITY=50 \
bash scripts/observe_lane_web.sh
```

Khi bật tùy chọn này, web viewer có hai trang, chuyển bằng nút ở phía trên:

**Trang Segmentation**:

1. **Raw Camera** — ảnh đầu vào và ánh sáng thực tế;
2. **Processor ROI** — vùng `40–63%` mà processor sử dụng;
3. **Yellow Mask** — pixel vàng và dữ liệu ứng viên boundary;
4. **Otsu Dark Mask** — mask đường đen trước corridor;
5. **Corridor Mask** — vùng lane sau boundary/recovery;
6. **Final Scan Mask** — chính xác mask mà `scan_row()` nhận.

Khi bật Segmentation, trước khi arm hai trang phải cập nhật liên tục; đặc biệt
`Final Scan Mask` phải chỉ chứa vùng lane hợp lệ và không có scanband kéo gần
toàn chiều rộng ảnh. Ở chế độ mặc định, kiểm tra liên tục bốn panel của
`Tracking / Output`. Không có panel đứng hình hoặc ảnh lỗi encoding.

## Terminal 7 — pre-arm checks, chạy một lần

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash

ros2 topic echo /line/status --once
ros2 topic echo /lane/reference --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /nmpc/status --once
ros2 topic echo /control/nmpc --once
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
ros2 topic info -v /control/nmpc
ros2 topic info -v /mcu/actuator_cmd
```

Chỉ được arm khi:

- `/line/status` là `line_detected`;
- `/lane/reference` có `status=1`, mảng hợp lệ và timestamp mới;
- `/odometry/filtered` có dữ liệu mới, tốc độ gần `0` khi xe đứng yên;
- `/nmpc/status` là `ACTIVE_VALID`;
- `/control/nmpc` chỉ có publisher NMPC dự kiến;
- `/mcu/actuator_cmd` chỉ có publisher `actuation_manager`;
- `/actuation/status` là `DISARMED`, servo `1500`, ESC `1550`;
- `/mcu/actuator_state` là `[1500,1550,flags]`.

Nếu thiếu một điều kiện, không arm.

## Terminal 8 — arm và quan sát thủ công

Chỉ chạy sau khi operator xác nhận xe, map, E-stop và pre-arm đều an toàn:

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: true}"
```

Theo dõi trực tiếp:

```bash
ros2 topic echo /nmpc/status --field data
ros2 topic echo /actuation/status --field data
ros2 topic echo /mcu/actuator_state
```

Trong lúc xe chạy, ghi nhớ hoặc nói thành tiếng các mốc: xuất phát, đường
thẳng, bắt đầu cua, giữa cua, ra cua, chuyển làn và safe-stop. Quan sát đồng
thời panel với log `CTE`, `heading`, `kappa`, `bands` và trạng thái NMPC.

Không arm lại sau `lane_invalid`, `solver_failed`, `command_invalid` hoặc
`cte_outside_safe_bound`. Khi có bất thường, disarm ngay.

## Terminal 9 — dừng an toàn

```bash
ros2 service call /actuation/arm \
  std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /actuation/status --once
ros2 topic echo /mcu/actuator_state --once
```

Kỳ vọng cuối: `DISARMED`, servo `1500`, ESC `1550`. Sau đó Ctrl+C theo thứ tự
Terminal 5, 4, 3, 2, 1, 6; chỉ dừng process do lượt này khởi động.

## Báo cáo thủ công sau lượt chạy

Không cần tạo file hay lưu dữ liệu trong quy trình này. Khi quay lại chat, báo
các thông tin sau:

- xe chạy được tới đoạn nào;
- panel nào có dấu hiệu sai đầu tiên;
- `bands`, `CTE`, `heading`, `kappa` ngay trước khi lỗi;
- NMPC status và actuation status khi lỗi xảy ra;
- xe dừng do operator hay do safe-stop tự động.

Kết quả lượt này chỉ dùng để định hướng fix. Không ghi nhận Gate 5 PASS nếu
không có evidence đầy đủ và ba vòng liên tiếp fault-free.
