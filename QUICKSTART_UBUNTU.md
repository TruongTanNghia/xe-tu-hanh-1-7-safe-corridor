# Chạy nhanh trên Ubuntu 22.04 + ROS 2 Humble

```bash
git clone https://github.com/TruongTanNghia/xe-tu-hanh-1-7-safe-corridor.git
cd xe-tu-hanh-1-7-safe-corridor
```

## 1. Demo offline (không cần ROS)

```bash
pip3 install numpy scipy pytest
bash scripts/corridor_offline_demo.sh            # SVG + summary.json vào evidence/
```

## 2. Demo real-time trong ROS 2 + RViz2 (không cần xe, không cần acados)

Mô phỏng xe chạy trên sa bàn oval, LiDAR giả, lane giả; stack corridor chạy
thật và xe tự lách vật cản theo hành lang.

```bash
source /opt/ros/humble/setup.bash
sudo apt install -y python3-colcon-common-extensions ros-humble-rviz2 python3-numpy python3-scipy
cd ros2_ws
colcon build --symlink-install --packages-select rc_interfaces rc_safe_corridor
source install/setup.bash
ros2 launch rc_safe_corridor sim_demo.launch.py scenario:=slalom
```

Kịch bản: `clear | right | left | slalom | curve | block | lead | mixed`.
Trong RViz2: vàng = biên làn, cam = LiDAR, đỏ = vật cản đã bám vết,
xanh = hành lang, trắng = d_ref, mũi tên tím = xe.

Xem trạng thái ở terminal khác:
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 topic echo /planning/status
ros2 topic echo /sim/status
```
Nếu RViz2 không mở được trong máy ảo: `export LIBGL_ALWAYS_SOFTWARE=1` rồi chạy lại.

## 3. Build đầy đủ cho xe thật

```bash
sudo apt install -y python3-vcstool
vcs import ros2_ws/src/vendor < dependencies.repos      # sllidar, rf2o, bno055, vision_opencv, micro-ROS
export RC_ROS_UNDERLAY=/opt/ros/humble/setup.bash
bash scripts/build.sh rc_interfaces rc_safe_corridor rc_bringup
# NMPC (race_cars) cần acados: scripts/bootstrap_acados.sh và docs/vi/step03_nmpc.md
```

Tài liệu đầy đủ: `docs/vi/step06_safe_corridor.md`; tài liệu định hướng của khách và
bản trình bày PDF trong `docs/customer/`.
