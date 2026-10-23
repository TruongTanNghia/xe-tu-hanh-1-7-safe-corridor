# Step 06 — Nhận thức vật cản & hành lang an toàn (Safe Corridor)

Triển khai theo tài liệu định hướng
`Dinh_huong_Safe_Corridor_xe_tu_hanh_1-7.pdf` (phiên bản 1.0, 14/10/2026).
Mục tiêu: xe tự nhận biết vật cản quanh mình bằng LiDAR, tự tạo hành lang an
toàn `[d_min(s), d_max(s)]` trong hệ Frenet bám tâm làn, và NMPC bám theo hành
lang đó. Lane Detection V2 và mô hình NMPC **giữ nguyên**; chỉ thêm các khối
mới và một lớp an toàn độc lập.

Trạng thái: **code đã viết, unit test offline PASS (44 + 5 test), chưa có
runtime evidence trên xe**. Mọi mốc nghiệm thu ở mục 7 vẫn phải chạy thật.

## 1. Kiến trúc

```text
/scan ──┬─► obstacle_detector (A) ──► /perception/obstacles ─┐
        │      ROI → lane gate → breakpoint cluster →        │
        │      fit tròn/hộp → Kalman CV (odom) → M/N confirm  │
        │                                                     ▼
/lane/reference ───────────────────────────► safe_corridor_planner (B+C+E)
   (+ evaluation_forward_m, path_lateral_m)     Frenet (s,d) · corridor · FSM
                                                     │
                              /planning/corridor ◄───┤──► /planning/behavior
                                     │               └──► /planning/corridor_markers
                                     ▼
                     racecar_nmpc_node (D, sửa) ──► /control/nmpc ──► rc_actuation
                        ràng buộc mềm theo stage,                        ▲
                        footprint 2 vòng tròn, d_ref, v_max              │
                                                                         │
/scan + /odometry/filtered + /control/nmpc + /mcu/arm ──► safety_monitor (F)
                              vùng phanh + watchdog ──► /safety/estop, gọi /actuation/arm=false
```

| Khối | Package / file | Trạng thái |
| --- | --- | --- |
| A. Vật cản LiDAR | `rc_safe_corridor/scan_clustering.py`, `tracking.py`, `obstacle_detector_node.py` | code + test |
| B. Frenet | `rc_safe_corridor/frenet.py` | code + test |
| C. Hành lang | `rc_safe_corridor/corridor.py` | code + test |
| D. NMPC | `race_cars/bicycle_model.py`, `acados_settings.py`, `corridor_lookup.py`, `nmpc_node.py` | code + test helper; cần build acados trên Pi |
| E. FSM | `rc_safe_corridor/behavior_fsm.py` | code + test |
| F. Safety monitor | `rc_safe_corridor/safety_zone.py`, `safety_monitor_node.py` | code + test |
| G. Camera vật cản | chưa làm (mở rộng, GĐ5) | — |

## 2. Message mới (`rc_interfaces`)

- `Obstacle` / `ObstacleArray`: id, tâm `(x, y)` trong `base_link`, `yaw`,
  `length/width/radius`, `vx/vy`, hiệp phương sai 2x2, `is_dynamic`,
  `confirmed`, `confidence`, `shape` (0 tròn, 1 hộp), `point_count`.
- `Corridor`: `s[]`, `lower[]`, `upper[]` (biên cho **tâm xe**, đã trừ `w/2`
  và lề), `d_ref[]`, `v_max[]`, `kappa_ref[]`, `ego_d`, `side`
  (0 giữa / 1 trái / 2 phải), `feasible`, `blocked_at_s`, `behavior_state`,
  `v_ref`. `s = 0` tại hình chiếu của `base_link` lên tâm làn; `d` dương về
  bên trái, trùng dấu với trạng thái `n` của NMPC.
- `BehaviorState`: `LANE_FOLLOW / AVOID / FOLLOW / STOP / ESTOP`, lý do,
  `v_ref`, `lead_gap_m`, `lead_speed_mps`, `avoid_side`.
- `LaneReference` thêm 2 trường tùy chọn `evaluation_forward_m`,
  `path_lateral_m[]` (adapter điền từ `/line/evaluation_forward` và
  `/line/reference_path`). NMPC cũ không đọc các trường này.
- `ControlCommand` thêm `REASON_CORRIDOR_STALE=9`, `REASON_CORRIDOR_BLOCKED=10`,
  `REASON_SAFETY_STOP=11`.

## 3. Quy ước hình học

- `base_link`: +x tiến, +y trái. LiDAR tại `x=-0.005, z=0.19`
  (`sensor_extrinsics.yaml`), đọc qua TF `base_link→laser`, fallback tham số.
- Xe: dài `0.632 m` (`footprint_front_m = footprint_rear_m = 0.32`), rộng
  `0.30 m` tính cả bánh, wheelbase `0.375 m`.
- Sa bàn: làn `0.45 m`, hai làn `0.90 m`; vùng được phép đi
  `lane_half_width = 0.45` trùng `model.cte_limit_m`. Khoảng trống cho tâm xe
  khi không có vật cản: `±(0.45 − 0.15 − 0.03) = ±0.27 m`.
- Vật cản phồng: `w/2 + margin_obstacle (0.08)` theo `d`;
  `overhang (0.32) + margin_s (0.10)` theo `s`. Vật nhỏ hơn
  `min_obstacle_size (0.10)` được nới lên kích thước này vì LiDAR chỉ thấy 1–2
  mặt.

## 4. Thuật toán chính

**A. Detector** (`obstacle_detector`, tần số = LiDAR ≈ 6 Hz)
1. Cắt điểm trong hộp thân xe, ROI `0.12 < x ≤ 2.5`, `|y| ≤ 1.0`.
2. Lane gate: giữ điểm `|d| ≤ 0.45 + 0.15` so với tâm làn dựng từ
   `/lane/reference`; khi lane stale → chỉ dùng ROI.
3. Adaptive breakpoint (Borges & Aldon, λ=10°, σ_r=1 cm), cụm ≥3 điểm, bỏ
   cụm dài hơn `1.2 m` (tường/mép bàn).
4. Vật ≤ 12 cm → đường tròn; dài hơn → hộp theo PCA.
5. Kalman CV `[x, y, vx, vy]` trong `odom` (tự về `base_link` nếu odom
   stale), ghép Hungarian + gating Mahalanobis, confirm sau 3 hit, xoá sau 5
   miss, `is_dynamic` khi `|v| > 0.15 m/s` (trễ 3 khung, tắt dưới 0.08).

**B+C. Planner** (`safe_corridor_planner`, 20 Hz)
1. Dựng tâm làn từ `s_m`, `kappa_s`, `cte`, `heading`, `evaluation_forward`,
   `path_lateral`; ngoại suy thẳng về sau đến gốc xe và ngoại suy trước bằng
   cung tròn `κ_end` tới `horizon 2.5 m`; lọc EMA `α=0.6`.
2. Chiếu chu vi vật cản (đỉnh hộp + trung điểm cạnh, hoặc 12 điểm trên vòng
   tròn) sang `(s, d)`; vận tốc chiếu thành `(v_s, v_d)`.
3. Hành lang: `F_i = [−0.27, +0.27]` trừ các dải cấm; vật động dịch theo
   `t_i = s_i / v_ego` và phồng thêm `0.10 m/s · t_i`.
4. Chọn phía vượt bằng DP: chi phí nút `−α·rộng + β·|tâm|`, cạnh chỉ nối
   khi chồng lấn ≥ 4 cm, phạt đổi phía `γ`, thưởng giữ phía chu kỳ trước.
5. Hậu xử lý: phễu giới hạn độ dốc ngang `0.45` (≈24°), `d_ref` lệch tối
   thiểu khỏi tâm làn trộn 50 % với tâm hành lang và làm mượt 7 trạm,
   `v_max` giảm theo bề rộng, theo `κ` (`a_lat ≤ 1.5`), và profile phanh
   `√(2·a·Δs)` trước điểm `blocked_at_s − 0.25`.
6. Xe đi trước cùng chiều (động, trong làn, `v_s ≥ 0.1`) **không** bị trừ
   khỏi hành lang; FSM xử lý bằng luật ACC
   `v = min(v_cruise, v_lead + 0.8·(gap − (0.5 + 1.0·v_lead)))`.

**E. FSM**: dwell tối thiểu 0.5 s; `AVOID → LANE_FOLLOW` sau 1.0 s thông;
`STOP → *` sau 1.5 s thông; mất lane > 0.5 s → `STOP`; `/safety/estop` →
`ESTOP` giữ tới khi được reset.

**D. NMPC** (`racecar_nmpc_node`, 20 Hz, `corridor_mode: optional`)
- Mỗi chu kỳ lấy `s_k` dự báo từ nghiệm trước (dịch lùi `v·T`), tra
  `lower/upper/d_ref/v_max/kappa` theo từng stage; nạp
  `constraints_set(k, "lh"/"uh")` cho hàng `n`, `n_front = n + 0.32·sin α`,
  `n_rear = n − 0.32·sin α` (ràng buộc mềm, slack L1 200 / L2 500) và
  `yref` với `n_ref = d_ref(s_k)`, `v_ref = min(target, v_max(s_k), v_ref_FSM)`.
- Thiếu corridor: `optional` → quay về biên làn `±0.45` như trước;
  `required` → dừng với `REASON_CORRIDOR_STALE`. `ESTOP` → dừng với
  `REASON_SAFETY_STOP`. Slack > `0.15 m` trong 5 chu kỳ liên tiếp → dừng
  với `REASON_CORRIDOR_BLOCKED`.
- `corridor_mode: off` cho hành vi y hệt bản trước (chỉ thêm 2 hàng ràng buộc
  bằng biên làn, không ảnh hưởng nghiệm).
- Vì thêm hàng ràng buộc, chữ ký cache acados có `ocp_variant`; lần chạy đầu
  sẽ sinh lại mã C.

**F. Safety monitor** (`safety_monitor`): vùng phanh
`D = v²/(2·1.5) + 0.25·v + 0.20` (clamp 0.25–2.0 m) × rộng `2·0.17 m` trước
mũi xe; ≥3 điểm trong vùng khi đang ARMED → latch `/safety/estop=true`
(transient-local) và gọi `/actuation/arm false`. Watchdog `/control/nmpc`
200 ms và `/scan` 600 ms khi ARMED. Reset bằng `ros2 service call
/safety/reset std_srvs/srv/Trigger`.

## 5. Lệnh

```bash
# Build (Pi)
bash scripts/build.sh rc_interfaces rc_safe_corridor race_cars rc_bringup
bash scripts/test.sh rc_safe_corridor race_cars

# Demo offline, không cần ROS (máy phát triển hoặc Pi)
bash scripts/corridor_offline_demo.sh              # SVG + summary.json vào evidence/

# Demo real-time ROS 2 + RViz2, không cần xe/acados (sim_world: sa bàn oval,
# LiDAR + lane giả, xe tự lách theo hành lang)
ros2 launch rc_safe_corridor sim_demo.launch.py scenario:=slalom   # clear|right|left|slalom|curve|block|lead|mixed

# Chỉ stack corridor (sensors + NMPC đã chạy ở terminal khác)
bash scripts/run_safe_corridor.sh stack enable_disarm:=false

# Toàn bộ: sensors + lane + EKF + corridor + NMPC shadow
bash scripts/run_safe_corridor.sh full
# Active NMPC chỉ trong gate có record:
bash scripts/run_safe_corridor.sh full nmpc_shadow:=false corridor_mode:=required

# Quan sát
ros2 topic echo /perception/obstacle_status
ros2 topic echo /planning/status
ros2 topic echo /nmpc/corridor_status
ros2 topic echo /safety/status
# RViz2 (laptop, cùng DDS peer): Fixed Frame base_link, thêm MarkerArray
# /perception/obstacle_markers và /planning/corridor_markers, LaserScan /scan.
```

## 6. Tham số quan trọng (`rc_safe_corridor/config/safe_corridor_hardware.yaml`)

| Tham số | Mặc định | Ghi chú |
| --- | --- | --- |
| `roi_x_max` | 2.5 | tầm nhìn vật cản |
| `lane_gate_margin` | 0.15 | lề ngoài biên làn khi lọc điểm |
| `cluster_max_extent_m` | 1.2 | cụm dài hơn coi là tường |
| `track_confirm_hits` / `track_max_misses` | 3 / 5 | lọc vật "ma" |
| `corridor.margin_obstacle` | 0.08 | lề tới vật cản (ngoài `w/2`) |
| `corridor.max_lateral_slope` | 0.45 | phễu hành lang |
| `corridor.v_cruise` | 0.6 | phải trùng `target_speed_mps` của NMPC |
| `fsm.blocked_stop_distance_m` | 2.0 | `blocked_at_s` dưới mức này → STOP |
| `zone.a_brake` / `zone.d0` | 1.5 / 0.20 | vùng phanh safety monitor |
| NMPC `slack_stop_m` / `slack_stop_cycles` | 0.15 / 5 | ngưỡng vi phạm hành lang |

Bắt đầu test thật với `target_speed_mps = corridor.v_cruise = 0.3`, tăng dần.

## 7. Lộ trình & nghiệm thu (theo PDF mục 10)

| GĐ | Việc | Nghiệm thu | Trạng thái |
| --- | --- | --- | --- |
| 0 | TF chuẩn, đo trễ, ghi ≥20 bag có vật cản, vật cao hơn mặt quét LiDAR (0.19 m) ≥ 3–5 cm | RViz scan chồng đúng lên làn; bảng trễ | chưa |
| 1 | Khối A + F trên xe | ≥95 % vật tĩnh trong 2 m, sai số tâm ≤3 cm, ≤1 vật ma/phút; safety monitor dừng 10/10 lần | code xong, chưa test |
| 2 | B + C trên bag rồi online (xe vẫn chạy NMPC cũ, chỉ hiển thị) | hành lang đúng 100 % kịch bản bag, không đổi phía qua lại, ≤5 ms/chu kỳ | code + test offline |
| 3 | D + E, bắt đầu 0.3 m/s | S2 ≥9/10, S4 slalom, S5 dừng êm, solver p95 ≤15 ms | code xong, chưa test |
| 4 | Vật động, FOLLOW | bám xe trước dao động <10 cm, không va chạm 20 lượt | code CV-prediction + ACC, chưa test |
| 5 | Camera vật thấp (G1 IPM) + đánh giá | phát hiện vật thấp; bảng chỉ số mục 12 PDF | chưa |

Kịch bản S1–S5, S7 đã có bản offline trong
`rc_safe_corridor/test/test_scenarios.py` và `offline_demo.py`.

## 8. Rủi ro đã xử lý trong code

- Đường tham chiếu nhảy: EMA lane + neo `s=0` tại xe + ngoại suy cung tròn.
- Lưỡng lự trái/phải: thưởng giữ phía trong DP + dwell/clear timer FSM.
- Solver vô nghiệm: mọi ràng buộc hành lang đều mềm; slack là tín hiệu dừng.
- Vật ma do phản xạ: ROI + lane gate + M/N confirm.
- Xe cùng chiều phía trước: không trừ khỏi hành lang (tránh STOP nhầm) mà
  giữ khoảng cách bằng ACC; safety monitor là lưới cuối.
- Điểm mù LiDAR 2D (vật thấp hơn 0.19 m): **chưa** có, dành cho khối G.

## 9. Việc còn mở

- Build/test trên Pi với acados: xác nhận `constraints_set` theo stage và
  thời gian giải khi `nh = 7`, 4 slack.
- Hiệu chuẩn camera–LiDAR (đặt vật có cạnh rõ, so trên ảnh và scan).
- Thu bag có vật cản để tinh chỉnh `cluster_*`, `margin_*`.
- Khối G (camera vật thấp) và tham chiếu toàn cục B1-b nếu cần tầm nhìn dài
  hơn 2.5 m.
