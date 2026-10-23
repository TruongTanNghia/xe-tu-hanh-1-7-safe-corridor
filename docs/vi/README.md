# Tài liệu vận hành V2

Đọc [`project_state.md`](../../project_state.md) trước. Đây là snapshot
canonical để các phiên làm việc AI tiếp tục đúng từ trạng thái hiện tại.

Đọc theo mục tiêu:

1. [`architecture.md`](architecture.md) — luồng dữ liệu, ownership, frame/topic.
2. [`operator_runbook.md`](operator_runbook.md) — quy trình test theo gate.
3. [`step01_image_processing.md`](step01_image_processing.md) — camera/lane
   hiện tại.
4. [`run_lane_detection.md`](run_lane_detection.md) — chạy và lưu lane evidence.
5. [`lidar_remote_visualization.md`](lidar_remote_visualization.md) — quan sát
   LiDAR qua SSH.
6. [`step02_fusion.md`](step02_fusion.md) — LiDAR/RF2O + BNO055 tùy chọn + EKF.
7. [`step03_nmpc.md`](step03_nmpc.md) — NMPC shadow và Gate 2 v2 model-fit.
8. [`step04_microros_actuation.md`](step04_microros_actuation.md) — actuation
   fail-safe.
9. [`step05_sensor_extrinsics.md`](step05_sensor_extrinsics.md) — calibration
   bắt buộc.
10. [`test_checklist.md`](test_checklist.md) — checklist ký gate.
11. [`ai_workflow/prompt_gate1.md`](ai_workflow/prompt_gate1.md) — prompt và
    trạng thái chuyển tiếp/lịch sử Gate 2 v2 → Gate 3.
12. [`ai_workflow/prompt_gate3.md`](ai_workflow/prompt_gate3.md) — prompt khởi
    động chat mới để thực hiện Gate 3.
13. [`command_teminal.md`](command_teminal.md) — cheat sheet lệnh terminal
    theo workflow Gate 2 v2 và liên kết tới lệnh Gate 5.
14. [`migration_audit.md`](migration_audit.md) và [`archive.md`](archive.md) —
    phần giữ lại, phần loại khỏi runtime.
15. [`gate5_manual_test.md`](gate5_manual_test.md) — lệnh chạy Gate 5 full map
    theo từng terminal, điều kiện pre-arm, record rosbag và dừng an toàn.
16. [`ai_workflow/prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md)
    — prompt chat mới cho diagnostic image processing + servo-only.
17. [`gate5_manual_live_no_record.md`](gate5_manual_live_no_record.md) — full-loop
    exploratory qua web panel, không ghi rosbag/CSV.
18. [`step06_safe_corridor.md`](step06_safe_corridor.md) — nhận thức vật
    cản LiDAR, hành lang an toàn Frenet, FSM hành vi, NMPC ràng buộc hành
    lang và safety monitor (package `rc_safe_corridor`, 2026-10-22).

Trạng thái mới nhất 2026-07-16: Gate 5 test 1, 2 và 3 đều chưa PASS. Test 3
dùng exposure `4000 us`, vượt chuyển làn đơn nhưng safe-stop khi vào cua gắt;
xem [`evidence/test3_gate5.mp4`](../../evidence/test3_gate5.mp4), report test 3
và [`report lượt cuối`](../../evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716/report.md).

`image_processing_lane_detection_notes.md` chỉ là ghi chú nghiên cứu, không phải
nguồn parameter. Mọi tài liệu cũ không còn đúng phải được cập nhật hoặc đưa vào
archive, không dùng song song với runbook canonical.
