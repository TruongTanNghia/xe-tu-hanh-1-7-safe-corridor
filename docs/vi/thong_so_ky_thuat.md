# Thông số kỹ thuật dùng để xây dựng Safe Corridor

Tổng hợp mọi con số đã dùng khi thiết kế, cài đặt và kiểm thử. Cột "Nguồn"
cho biết file chứa giá trị đang chạy; muốn đổi thì sửa ở file đó, tài liệu
này chỉ để tra cứu.

## 1. Phần mềm và môi trường

| Hạng mục | Phiên bản / giá trị | Ghi chú |
| --- | --- | --- |
| Hệ điều hành xe | Raspberry Pi OS (Debian 12) arm64 trên Pi 5 8 GB | ROS build từ source tại `/home/ac/ros2_humble` |
| Hệ điều hành mô phỏng | Ubuntu 22.04 | ROS cài apt tại `/opt/ros/humble` |
| ROS 2 | Humble | |
| Gazebo | Classic 11 (`gazebo_ros_pkgs`, `gazebo_ros2_control`) | chỉ cho mô phỏng |
| Python | 3.10 (Ubuntu 22.04), đã test thêm 3.11 | |
| Thư viện Python | numpy ≥ 1.21, scipy ≥ 1.8, matplotlib (chỉ `sim_view`) | scipy cho Hungarian; thiếu scipy thì tự dùng ghép tham lam |
| NMPC | acados (SQP_RTI + HPIPM partial condensing), CasADi | `/home/ac/acados` trên xe |
| Bộ đo kiểm | pytest | 44 test `rc_safe_corridor` + 5 test `corridor_lookup` |

## 2. Xe 1/7 (MJX Hyper Go 7304)

| Thông số | Giá trị | Nguồn |
| --- | --- | --- |
| Chiều dài / rộng thân | 0,632 m / 0,210 m (0,30 m tính cả bánh) | `rc_1_7.urdf.xacro` |
| Chiều dài cơ sở (wheelbase) | 0,375 m | URDF, NMPC |
| Vệt bánh (track width) | 0,260 m | URDF |
| Bán kính bánh | 0,049 m | URDF |
| Khối lượng | 6,5 kg | URDF, `model.mass_kg` |
| Gốc `base_link` | giữa hai cầu, ngang trục bánh; +x tiến, +y trái, +z lên | `sensor_extrinsics.yaml` |
| Mũi / đuôi tính từ `base_link` | 0,32 m / 0,32 m | `footprint_front_m`, `front_overhang` |
| Góc lái tối đa | ±0,524 rad (±30°), đo thật 1150–1850 µs | `actuation_hardware.yaml` |
| Tốc độ góc lái tối đa | ±2,5 rad/s | NMPC |
| Servo | giữa 1500 µs, 668,45 µs/rad | `actuation_hardware.yaml` |
| ESC | dừng 1550 µs, bắt đầu chạy 1580 µs, tối đa 1600 µs, tăng 50 µs/s | `actuation_hardware.yaml` |
| Tần số actuation | 50 Hz, timeout lệnh 0,2 s, timeout MCU 0,75 s | `actuation_hardware.yaml` |

## 3. Cảm biến

| Cảm biến | Thông số | Nguồn |
| --- | --- | --- |
| LiDAR RPLIDAR A1M8-R6 | 360°, tầm 0,15–12 m, ~6–10 Hz, tam giác lượng, UART 115200 | `fusion_hardware.launch.py` |
| Vị trí LiDAR | `base_link → laser`: x = −0,005, y = 0, z = 0,190 m, yaw = 0 (đã đo, `measured_ccw_verified`) | `sensor_extrinsics.yaml` |
| Mặt quét LiDAR so với mặt sa bàn | ≈ 0,239 m (0,190 + bán kính bánh 0,049) | vật cản thử phải cao hơn |
| Camera IMX708 Wide | 120° FOV, chạy 640×360 bgr8 30 fps | `hardware_camera` |
| Nội tham số (ảnh gốc 4608×2592) | fx 1968,93, fy 1968,09, cx 2299,32, cy 1296,28; méo k1 −0,0589, k2 0,0996 | `imx708_wide_4608x2592.yaml` |
| Vị trí camera | x = 0,276, y = 0, z = 0,160 m (tạm, chưa hiệu chuẩn cuối) | `lane_detection.launch.py` |
| IMU BNO055 | `base_link → imu_link`: z = 0,100 m | `sensor_extrinsics.yaml` |
| Odometry | RF2O từ LiDAR → lọc median 3 + EMA 0,45 → EKF 50 Hz (`/odometry/filtered`) | `velocity_conditioner.yaml`, `ekf_hardware.yaml` |

## 4. Sa bàn

| Thông số | Giá trị |
| --- | --- |
| Kích thước in sa bàn thật | 6,78 × 2,95 m, chuyển làn offset 450 mm, cua R700 / R925, đoạn chuyển tiếp 1000 mm |
| Vùng được phép đi quanh tâm làn | ±0,45 m (`lane_half_width`, trùng `model.cte_limit_m`) |
| Sa bàn mô phỏng (`sim_world`, Gazebo) | oval hai làn 2 × 0,45 m, hai đoạn thẳng 3,5 m, cua bán kính tâm 0,925 m, chu vi 12,81 m |
| Vật cản thử | cọc tròn bán kính 0,05 m, hộp chắn 0,2 × 0,9 m, xe trước 0,3 × 0,2 m; cao 0,40 m trong Gazebo |

## 5. Lane Detection V2 (giữ nguyên)

| Tham số | Giá trị |
| --- | --- |
| Vùng quét ảnh | 40–63 % chiều cao, 0–100 % chiều rộng, 12 dải quét cao 6 px |
| Ngưỡng tối / số dải tối thiểu | 60 / 6 |
| Phục hồi biên | bật, tối đa 150 ms, tối thiểu 3 quan sát |
| Đầu ra cho Safe Corridor | `/lane/reference`: `s_m[]`, `curvature_inv_m[]`, `cte_m`, `heading_error_rad`, `confidence` + mới `evaluation_forward_m`, `path_lateral_m[]` |

## 6. Khối A: nhận vật cản LiDAR (`obstacle_detector`)

| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| ROI | 0,12 < x ≤ 2,5 m, \|y\| ≤ 1,0 m | vùng quan tâm phía trước |
| Hộp thân xe loại bỏ | \|x\| ≤ 0,40, \|y\| ≤ 0,18 m | bỏ điểm dây, cột camera |
| Lọc theo làn | \|d\| ≤ 0,45 + 0,15 m | bỏ tường, chân người ngoài sa bàn |
| Adaptive breakpoint | λ = 10°, σr = 0,01 m, cụm ≥ 3 điểm | Borges & Aldon 2004 |
| Cụm dài tối đa | 1,2 m | dài hơn coi là tường |
| Vật nhỏ → tròn | kích thước ≤ 0,12 m, bán kính tối thiểu 0,03 m | dài hơn → hộp PCA |
| Kalman CV | nhiễu gia tốc 1,5 m/s², nhiễu đo 0,03 m, khung `odom` | |
| Ghép track | Hungarian, cổng Mahalanobis χ² 9,21, cổng khoảng cách 0,5 m | |
| Vòng đời track | xác nhận sau 3 lần thấy, xoá sau 5 lần mất hoặc 0,8 s | lọc vật "ma" |
| Động / tĩnh | động khi \|v\| > 0,15 m/s trong 3 khung, về tĩnh khi < 0,08 m/s | |

## 7. Khối B + C: Frenet và hành lang (`safe_corridor_planner`)

| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| Tần số | 20 Hz | |
| Bước s / tầm nhìn | 0,05 m / 2,5 m (51 trạm) | |
| Lọc lane | EMA α = 0,6 | chống hành lang giật |
| Ngoại suy tâm làn | cung tròn theo κ cuối (phía trước), κ đầu (phía sau) | |
| Bề rộng xe dùng phồng | 0,30 m | |
| Lề tới vạch | 0,03 m → khoảng trống tâm xe ±0,27 m khi không có vật | |
| Lề tới vật cản (ngang) | w/2 + 0,08 m | |
| Lề dọc | mũi/đuôi 0,32 m + 0,10 m | |
| Kích thước vật tối thiểu | 0,10 m | LiDAR chỉ thấy 1–2 mặt |
| Bề rộng hành lang tối thiểu / chồng lấn | 0,02 m / 0,04 m | |
| Độ dốc ngang tối đa của phễu | 0,45 (≈ 24°) | khả năng bẻ lái |
| Chi phí DP | rộng 1,0; lệch tâm 2,0; nhảy ngang 1,0; đổi phía 1,5; thưởng giữ phía 0,6 | chọn phía vượt |
| d_ref | lệch tối thiểu khỏi tâm làn + 50 % tâm hành lang, làm mượt 7 trạm, lề 0,05 m | |
| Tốc độ | v_cruise 0,6 m/s; ≥ 40 % khi hẹp; a_lat ≤ 1,5 m/s²; phanh 1,0 m/s²; dừng trước điểm chặn 0,25 m | |
| Vật động | dự đoán CV tối đa 2,0 s, phồng thêm 0,10 m/s, cấm cả vùng quét | |

## 8. Khối E: máy trạng thái hành vi

| Tham số | Giá trị |
| --- | --- |
| Trạng thái | LANE_FOLLOW, AVOID, FOLLOW, STOP, ESTOP |
| Thời gian giữ tối thiểu | 0,5 s |
| Thông đường để thoát AVOID / FOLLOW | 1,0 s |
| Thông đường để thoát STOP | 1,5 s |
| Mất lane → STOP | 0,5 s |
| Chặn trong khoảng → STOP | ≤ 1,2 m |
| Luật ACC (FOLLOW) | v = min(v_cruise, v_lead + 0,8·(gap − (0,5 + 1,0·v_lead))) |
| Nhận xe trước | động, trong làn (lệch ≤ 0,45 − 0,25 m), v_s ≥ 0,10 m/s, không cắt ngang |

## 9. Khối D: NMPC (`race_cars`)

| Tham số | Giá trị |
| --- | --- |
| Mô hình | xe đạp động học trong hệ đường cong, trạng thái [s, n, α, v, D, δ], điều khiển [Ḋ, δ̇] |
| Tham số dọc (fit Gate 2) | cm1 0,7208, cm2 1,7720, cr0 0,0763, cr2 0,4506, cr3 20,0, c1 0,5, c2 15,5 |
| Chân trời | 1,3 s, 10 bước (Ts 0,13 s), chạy 20 Hz |
| Trọng số stage | [0,5; 1,0; 5,0; 0,5; 1e−3; 1e−5], điều khiển [3e−4; 5e−4] |
| Trọng số cuối | [2,0; 30,0; 10,0; 0,5; 1e−3; 1e−9] |
| Giới hạn | D ∈ [0, 1], Ḋ ∈ ±12, δ ∈ ±0,524, δ̇ ∈ ±2,5, a_long và a_lat ∈ ±2,0 m/s² |
| Ràng buộc hành lang (mới) | hàng n, n_front = n + 0,32·sin α, n_rear = n − 0,32·sin α; mềm, slack L1 200 / L2 500 |
| Tra hành lang | theo s dự báo của nghiệm trước, nạp `lh/uh` từng stage |
| An toàn | slack > 0,15 m trong 5 chu kỳ → dừng; thiếu corridor quá 0,3 s → dừng (chế độ `required`) |
| `corridor_mode` | `off` (như bản cũ), `optional` (mặc định), `required` |

## 10. Khối F: safety monitor

| Tham số | Giá trị |
| --- | --- |
| Vùng phanh | D = v²/(2·1,5) + 0,25·v + 0,20 m, giới hạn 0,25–2,0 m, rộng 2 × 0,17 m, bắt đầu từ mũi xe 0,32 m |
| Kích hoạt | ≥ 3 điểm LiDAR trong vùng khi đang ARMED |
| Watchdog | lệnh `/control/nmpc` 0,2 s, `/scan` 0,6 s |
| Odom mất | giả định 0,8 m/s để vùng phanh dài hơn |
| Hành động | chốt `/safety/estop = true`, gọi `/actuation/arm false`; reset bằng `/safety/reset` |

## 11. Mô phỏng

| Hạng mục | Giá trị |
| --- | --- |
| LiDAR giả (`sim_world`, `sim_view`) | 1080 tia 360°, nhiễu 5 mm, 10 Hz |
| LiDAR Gazebo | ray sensor 720 tia, 0,15–12 m, nhiễu 5 mm, 10 Hz |
| Bộ điều khiển mô phỏng | `corridor_follower`: nhìn trước 0,5 m, hệ số hướng 1,8, bù cong atan(κ·L) |
| Gazebo | bước vật lý 2 ms, ros2_control 60 Hz, `ackermann_cmd_bridge` 50 Hz, gia tốc tối đa 1,0 m/s² |

## 12. Tài liệu tham khảo dùng khi thiết kế

- Werling et al., Optimal Trajectory Generation in a Frenét Frame, ICRA 2010.
- Fan et al., Baidu Apollo EM Motion Planner, arXiv:1807.08048, 2018.
- Liniger, Domahidi, Morari, Optimization-based autonomous racing of 1:43 scale RC cars, 2015.
- Verschueren et al., acados, Mathematical Programming Computation, 2022.
- Borges & Aldon, Line extraction in 2D range images for mobile robotics, 2004.
- Zhang et al., Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners, IV 2017.
- Tài liệu định hướng của khách: `docs/customer/Dinh_huong_Safe_Corridor_xe_tu_hanh_1-7.pdf`.
