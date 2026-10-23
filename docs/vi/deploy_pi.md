# Đưa code xuống xe (Raspberry Pi 5, ROS 2 Humble native)

Code trên repo là code chạy thật trên xe. Bộ mô phỏng (`sim_world`, `sim_view`)
chỉ thay cảm biến; các node `obstacle_detector`, `safe_corridor_planner`,
`safety_monitor` và NMPC chạy y hệt trên Pi.

## 1. Lấy code về Pi (một lần)

```bash
ssh ac@<ip-pi>
cd ~
git clone https://github.com/TruongTanNghia/xe-tu-hanh-1-7-safe-corridor.git v2_safe_corridor
cd v2_safe_corridor
# vendor (sllidar, rf2o, bno055, vision_opencv, micro-ROS): copy từ workspace cũ
# để khỏi tải lại, hoặc vcs import như bên dưới
cp -r ~/main/1_projects/autonomous_car_hardware_platform_v2/ros2_ws/src/vendor ros2_ws/src/ \
  || vcs import ros2_ws/src/vendor < dependencies.repos
```

Lần sau chỉ cần `git pull`.

## 2. Build (underlay mặc định của scripts là /home/ac/ros2_humble)

```bash
bash scripts/build.sh                   # toàn workspace, lần đầu 10-20 phút
# hoặc chỉ phần thay đổi:
bash scripts/build.sh rc_interfaces rc_safe_corridor race_cars rc_bringup
bash scripts/test.sh rc_safe_corridor race_cars
```

NMPC dùng acados tại `/home/ac/acados` (`RC_ACADOS_INSTALL`); lần chạy đầu
`racecar_nmpc_node` sinh lại mã C vì OCP có thêm ràng buộc hành lang (chờ ~1 phút).

## 3. Kiểm tra từng lớp, xe kê lên (wheels-up) hoặc giữ chắc

Terminal 1 – sensors + lane + EKF + corridor + NMPC **shadow** (không actuator):
```bash
bash scripts/run_safe_corridor.sh full
```
Terminal 2 – xác nhận các topic sống:
```bash
source scripts/ros_network_env.sh 2>/dev/null; source /home/ac/ros2_humble/install/setup.bash; source ros2_ws/install/setup.bash
ros2 topic hz /scan /lane/reference /planning/corridor
ros2 topic echo /perception/obstacle_status      # points/clusters/tracks, tf=tf (không phải param)
ros2 topic echo /planning/status                 # state, feasible, side, plan_ms
ros2 topic echo /nmpc/corridor_status            # corridor_active
ros2 topic echo /safety/status                   # latched=False, armed=False
```
Đặt một vật cao > 22 cm (cao hơn mặt quét LiDAR 19 cm) trước xe 1 m, lệch
phải: `/perception/obstacles` phải có 1 track `confirmed`, `/planning/status`
đổi sang `state=AVOID side=1`. Nhấc vật đi: về `LANE_FOLLOW` sau ~1 s.

RViz2 trên laptop cùng mạng (CycloneDDS peer như hướng dẫn cũ): Fixed Frame
`base_link`, thêm `/scan`, `/perception/obstacle_markers`, `/planning/corridor_markers`.

## 4. Safety monitor trước khi active (bắt buộc, 10/10 lần)

Terminal 3: actuation DISARMED như runbook cũ (`bash scripts/run_micro_ros_actuation.sh`),
arm thử với xe được giữ chắc. Đẩy vật vào trước mũi xe trong 0,3 m:
`/safety/status` phải chuyển `latched=True reason=brake_zone...` và actuation
tự DISARM. Reset bằng:
```bash
ros2 service call /safety/reset std_srvs/srv/Trigger
```

## 5. Chạy active lần đầu

```bash
# target_speed_mps và corridor.v_cruise cùng 0.3 cho lượt đầu (sửa 2 file yaml)
bash scripts/run_safe_corridor.sh full nmpc_shadow:=false corridor_mode:=required
```
Thứ tự kịch bản: không vật cản (S1) → 1 vật lệch phải (S2) → vật trong cua
(S3) → slalom (S4) → chắn kín làn (S5). Ghi rosbag mỗi lượt; đạt rồi mới tăng
tốc 0,4 → 0,6 m/s. Chi tiết tham số và nghiệm thu: `step06_safe_corridor.md`.

## 6. Điểm khác biệt mô phỏng ↔ xe thật cần để ý

| Mô phỏng | Xe thật |
| --- | --- |
| LiDAR không nhiễu phản xạ | vạch/băng keo phản quang có thể tạo điểm giả → chỉnh `cluster_min_points`, `lane_gate_margin` |
| lane luôn hợp lệ, không trễ | camera trễ 60–120 ms, có khung mất lane → FSM về STOP sau 0,5 s |
| bộ điều khiển đơn giản bám d_ref | NMPC acados, slack > 0,15 m 5 chu kỳ → dừng |
| odom hoàn hảo | RF2O ~6 Hz → tracker tự về `base_link` khi odom stale |
