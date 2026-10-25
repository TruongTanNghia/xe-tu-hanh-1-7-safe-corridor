# Mô phỏng Gazebo cho Safe Corridor

Gazebo Classic 11 + ROS 2 Humble, Ubuntu 22.04. Xe 1/7 lấy từ `rc_description`
(repo research), gắn thêm LiDAR RPLIDAR A1 đúng vị trí lắp trên xe thật
(`base_link → laser`: x = −0,005 m, z = 0,190 m). Sa bàn hai làn (2 × 0,45 m),
hai đoạn thẳng 3,5 m và hai cua bán kính tâm 0,925 m, có vật cản theo kịch bản.

```text
Gazebo ray sensor ──/scan──► obstacle_detector ─► safe_corridor_planner ─► /planning/corridor
ground truth ──► gazebo_corridor_bridge ──► /odometry/filtered, /lane/reference, TF odom→base_footprint
/planning/corridor ─► corridor_follower ─/cmd_vel─► ackermann_cmd_bridge ─► ros2_control (lái + bánh sau)
safety_monitor chạy song song (không ngắt arm vì không có phần cứng)
```

Các node `obstacle_detector`, `safe_corridor_planner`, `safety_monitor` là
đúng các node chạy trên xe thật. Khác biệt duy nhất: lane lấy từ hình học sa
bàn (ground truth) thay cho camera, và bộ điều khiển là `corridor_follower`
(không cần acados) thay cho NMPC.

## 1. Cài đặt (một lần)

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 python3-colcon-common-extensions python3-numpy python3-scipy
```

## 2. Build (hai workspace, `sim_ws` chồng lên `ros2_ws`)

```bash
cd ~/xe-tu-hanh-1-7-safe-corridor && git pull
source /opt/ros/humble/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select rc_interfaces rc_safe_corridor
source install/setup.bash

cd ../sim_ws
colcon build --symlink-install
source install/setup.bash
```

`sim_ws` để riêng để Raspberry Pi không phải cài Gazebo khi build `ros2_ws`.

## 3. Chạy

```bash
# mỗi terminal mới:
source /opt/ros/humble/setup.bash
source ~/xe-tu-hanh-1-7-safe-corridor/ros2_ws/install/setup.bash
source ~/xe-tu-hanh-1-7-safe-corridor/sim_ws/install/setup.bash

ros2 launch rc_gazebo_corridor gazebo_corridor.launch.py scenario:=tour
```

Mở cửa sổ Gazebo (xe, sa bàn, vật cản, tia LiDAR xanh) và RViz2 (scan, vật
cản đã bám vết, hành lang xanh, đường tham chiếu trắng). Khoảng 11 giây sau
khi mở, xe bắt đầu chạy.

| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `scenario` | `tour` (mặc định), `clear`, `right`, `left`, `slalom`, `curve`, `block`, `lead`, `mixed` | cách đặt vật cản |
| `gui` | `true` / `false` | mở cửa sổ Gazebo |
| `rviz` | `true` / `false` | mở RViz2 |
| `controller` | `follower` / `nmpc` | `nmpc`: không chạy follower, tự chạy NMPC (cần acados) |

Kịch bản `tour` (một vòng có đủ): vật lệch phải → vật trong cua → slalom 2
vật → người đi bộ băng qua khi xe tới gần → vật chắn kín làn (biến mất 6 s
sau khi xe dừng trước nó) → xe trước chạy chậm 0,25 m/s xuất hiện ở vòng sau.

## 4. Xem tín hiệu (terminal khác, đã source như trên)

```bash
ros2 topic hz /scan                              # LiDAR Gazebo ~10 Hz
ros2 topic echo /scan --once                     # dữ liệu thô như xe thật
ros2 topic echo /perception/obstacle_status      # điểm, cụm, track
ros2 topic echo /planning/status                 # LANE_FOLLOW / AVOID / FOLLOW / STOP
ros2 topic echo /sim/status                      # vị trí s, lệch d, tốc độ
ros2 topic echo /control/follower_status         # lệnh tốc độ, góc lái
```

## 5. Dùng NMPC thay cho follower (tùy chọn, cần acados)

```bash
ros2 launch rc_gazebo_corridor gazebo_corridor.launch.py scenario:=tour controller:=nmpc
# terminal khác:
ros2 run race_cars racecar_nmpc_node --ros-args \
  --params-file ros2_ws/src/race_cars/config/nmpc_hardware.yaml \
  -p use_sim_time:=true -p shadow_mode:=false -p corridor_mode:=required
```
NMPC phát `/nmpc_cmd = [góc lái, drive_effort]`, `ackermann_cmd_bridge` đổi
`drive_effort × 0,6 → m/s`. Mô hình dọc của NMPC được fit cho ESC thật nên
tốc độ trong Gazebo chỉ gần đúng; phần lái và ràng buộc hành lang thì giống.
Cần build `race_cars` trong `ros2_ws` và `export ACADOS_SOURCE_DIR=...`,
`LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH` trước.

## 6. Sự cố thường gặp

| Hiện tượng | Cách xử lý |
| --- | --- |
| Gazebo đen hoặc treo trong máy ảo | `export LIBGL_ALWAYS_SOFTWARE=1` rồi chạy lại; hoặc `gui:=false` và chỉ xem RViz |
| `spawn_entity` chờ mãi | còn `gzserver` cũ chạy: `killall -9 gzserver gzclient` |
| Xe không chạy, `/planning/status` = STOP lane_lost | kiểm tra `/odometry/filtered` và `/lane/reference` có dữ liệu |
| Không có `/scan` | thiếu `ros-humble-gazebo-plugins` (nằm trong `gazebo-ros-pkgs`) |
| Xe chạy chậm hơn 0,6 m/s | máy yếu làm Gazebo chạy dưới thời gian thực; logic không đổi |

Đã kiểm thử headless (Docker `ros:humble` + Gazebo 11) với `tour`: 17 node,
`/scan` 10 Hz, xe chạy 3 vòng, dừng trước vật chắn, bám xe trước, không lấn vạch.
