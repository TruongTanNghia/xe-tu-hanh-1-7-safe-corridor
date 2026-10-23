# Chạy nhanh trên Ubuntu 22.04 + ROS 2 Humble

```bash
git clone https://github.com/TruongTanNghia/xe-tu-hanh-1-7-safe-corridor.git
cd xe-tu-hanh-1-7-safe-corridor

# 1. Demo offline Safe Corridor (không cần ROS, chỉ cần python3 + numpy + scipy)
pip3 install numpy scipy pytest
bash scripts/corridor_offline_demo.sh            # xuất SVG + summary.json vào evidence/
cd ros2_ws/src/rc_safe_corridor && python3 -m pytest test -q && cd -

# 2. Build ROS 2 (ROS cài bằng apt tại /opt/ros/humble)
sudo apt install -y python3-vcstool python3-colcon-common-extensions
vcs import ros2_ws/src/vendor < dependencies.repos      # lấy sllidar, rf2o, bno055, vision_opencv, micro-ROS
export RC_ROS_UNDERLAY=/opt/ros/humble/setup.bash
bash scripts/build.sh rc_interfaces rc_safe_corridor rc_bringup
# NMPC (race_cars) cần acados: xem scripts/bootstrap_acados.sh và docs/vi/step03_nmpc.md

# 3. Chạy stack corridor với dữ liệu thật/bag (sensors chạy ở terminal khác)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch rc_safe_corridor safe_corridor.launch.py enable_disarm:=false
ros2 topic echo /planning/status
```

Tài liệu đầy đủ: `docs/vi/step06_safe_corridor.md`; tài liệu định hướng của khách và
bản trình bày PDF trong `docs/customer/`.
