# Quan sát LiDAR `/scan` khi dùng VS Code Remote SSH

## 1. Nguyên nhân đã tìm thấy trên Raspberry Pi

Ngày 2026-06-28, Pi có:

```text
wlan0 = 192.168.1.3/24
RMW_IMPLEMENTATION = rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY = 0
```

Nhưng `.bashrc` từng export:

```text
ROS_IP=192.168.0.5
```

Đây là địa chỉ thuộc subnet cũ và `ROS_IP` vốn là cách cấu hình ROS 1, không
phải cách chọn interface DDS chuẩn của ROS 2. CycloneDDS XML cũng chưa được
kích hoạt. Dòng này đã được loại bỏ và cấu hình mạng được chuyển vào project.

Ngày 2026-06-29, A1M8 đã được kiểm tra vật lý qua đường dẫn ổn định:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Pi nhận `/scan` frame `laser` ở khoảng `6,6 Hz`; một snapshot có `931/1080`
điểm hợp lệ. Publisher thực tế là **reliable/volatile**. Web viewer localhost
và rosbag replay ở domain cô lập đều PASS. Evidence có checksum:
`evidence/20260629-065611_lidar_live_a1m8`.

Đường DDS trực tiếp Pi → laptop chưa được thử với laptop trong session này,
vì vậy không được ghi là PASS. Đường live đã xác nhận là web viewer qua VS Code
SSH; fallback đã xác nhận là rosbag.

ROS 2 dùng DDS discovery. Cùng `ROS_DOMAIN_ID` chỉ là điều kiện cần; Wi-Fi có
thể chặn multicast giữa client, firewall có thể chặn UDP hoặc hai máy có thể
dùng khác RMW/address family. `LaserScan` thường dùng sensor-data QoS
best-effort; subscriber reliable sẽ không nhận publisher best-effort. A1M8
hiện tại publish reliable, nên subscriber best-effort của web viewer vẫn tương
thích.

Tài liệu chính:

- [ROS 2 Humble QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [CycloneDDS network interfaces](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)
- [CycloneDDS discovery peers](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html)
- [LaserScan definition](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/LaserScan.html)

## 2. Chuẩn bị

Không dừng camera hoặc process của người khác. Cắm LiDAR và kiểm tra:

```bash
ls -l /dev/serial/by-id /dev/ttyUSB* 2>/dev/null
ip -4 -brief address
```

Kiểm tra extension ghi bag. Nếu lệnh thứ nhất fail, build đúng nhóm rosbag2
trong source underlay rồi kiểm tra lại:

```bash
ros2 bag --help
bash scripts/bootstrap_rosbag2.sh
```

Khởi chạy driver LiDAR trong Terminal 1. Sau đó Terminal 2:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/lidar_network_diagnose.sh
```

Chỉ tiếp tục khi script báo Pi nhận được một message `/scan`.

## 3. Cách A — DDS trực tiếp vào RViz2 trên laptop

### 3.1 Kiểm tra multicast hai chiều

Laptop:

```bash
source /opt/ros/humble/setup.bash
ros2 multicast receive
```

Pi:

```bash
source /home/ac/ros2_humble/install/setup.bash
ros2 multicast send
```

Đảo vai trò và thử lần nữa. Nếu ping IPv4 được nhưng multicast không đi qua,
router/AP đang chặn hoặc xử lý multicast Wi-Fi không ổn. Đổi domain ID không
sửa được lỗi này.

### 3.2 Dùng CycloneDDS peer IPv4

Tìm IPv4 laptop trong cùng subnet `192.168.1.0/24`, ví dụ `192.168.1.20`.

Trên Pi, source trước khi chạy driver và các node cần chia sẻ:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source scripts/ros_network_env.sh 192.168.1.20
```

Trên laptop, copy file cấu hình:

```bash
scp ac@192.168.1.3:/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/config/dds/cyclonedds_laptop_peer.xml ~/cyclonedds_rc_car.xml
```

Sau đó:

```bash
source /opt/ros/humble/setup.bash
unset ROS_IP
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RC_DDS_PEER_IP=192.168.1.3
export CYCLONEDDS_URI=file://$HOME/cyclonedds_rc_car.xml

ros2 topic echo /scan --once --no-daemon --qos-profile sensor_data
```

Nếu lệnh echo nhận được dữ liệu, mở RViz2 trong chính terminal đó:

```bash
rviz2
```

Trong RViz:

1. `Fixed Frame = laser` nếu chưa có TF, hoặc `base_link` nếu TF đã chạy.
2. Add → `LaserScan`.
3. Topic `/scan`.
4. Reliability `Best Effort`, Durability `Volatile`.
5. Nếu thấy lỗi “No transform from laser”, sửa Fixed Frame trước khi sửa DDS.

Mọi process ROS 2 phải được khởi động sau khi export cấu hình. Không cần dừng
process của người khác; chỉ restart driver/viewer do mình vừa chạy.

## 4. Cách B — Web viewer qua SSH, không cần ROS trên laptop

Đây là đường ổn định nhất khi Wi-Fi chặn DDS multicast.

Build một lần:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2/ros2_ws
source /home/ac/ros2_humble/install/setup.bash
colcon build --symlink-install --packages-select rc_validation
```

Chạy trong terminal mới:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/run_lidar_web_viewer.sh
```

Trong VS Code:

1. Mở panel **PORTS**.
2. Forward port `8090`.
3. Mở `http://127.0.0.1:8090`.

Viewer subscribe `/scan` ngay trên Pi bằng sensor-data QoS và chỉ truyền JSON
qua SSH, nên không cần DDS giữa laptop và Pi.

## 5. Xuất dữ liệu và lưu bằng chứng

Lưu một scan thành SVG và JSON:

```bash
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
bash scripts/save_lidar_snapshot.sh
```

Mở file SVG trực tiếp bằng VS Code. Hướng `forward` nằm phía trên, trục trái xe
nằm bên trái hình.

Ghi rosbag bằng workflow chuẩn:

```bash
SESSION=$(bash scripts/evidence/new_session.sh --scenario lidar_live)
bash scripts/evidence/record.sh --session "$SESSION" --duration 30 -- \
  /scan /tf /tf_static
bash scripts/evidence/evaluate.sh --session "$SESSION" --mode lidar
bash scripts/evidence/finish_session.sh --session "$SESSION" --result PASS
```

Dừng bằng `Ctrl-C`. Copy bag sang laptop:

```bash
scp -r ac@192.168.1.3:/home/ac/main/1_projects/autonomous_car_hardware_platform_v2/evidence/<session>/rosbag/<bag> .
```

Laptop:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play <bag>
rviz2
```

Đây là fallback offline bắt buộc, kể cả khi DDS live đã hoạt động.

## 6. Đánh giá PASS/FAIL

PASS khi:

- Pi nhận `/scan` đúng type `sensor_msgs/msg/LaserScan`;
- rate ổn định và `valid_count > 0`;
- web viewer/SVG thể hiện vật cản thay đổi đúng khi đưa một vật trước LiDAR;
- ít nhất một trong DDS trực tiếp hoặc web viewer cho hình live trên laptop;
- rosbag replay được offline.

FAIL nếu:

- Pi không có `/scan`: lỗi driver/serial, chưa phải lỗi laptop;
- `/scan` có trên Pi nhưng multicast test fail: dùng peer hoặc web viewer;
- laptop echo được nhưng RViz trống: kiểm tra QoS và Fixed Frame;
- dữ liệu toàn `inf`/ngoài range: kiểm tra LiDAR, nguồn và môi trường quét.
