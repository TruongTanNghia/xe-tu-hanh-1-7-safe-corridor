# Prompt cho chat mới — bắt đầu Gate 3

Copy toàn bộ nội dung trong khung dưới đây để mở một AI chat mới và tiếp tục dự án.

```text
Bạn đang tiếp quản dự án tại:
/home/ac/main/1_projects/autonomous_car_hardware_platform_v2

Hãy đọc trước các tài liệu sau:
1. project_state.md
2. README.md
3. docs/vi/README.md
4. docs/vi/operator_runbook.md
5. docs/vi/step03_nmpc.md
6. docs/vi/step04_microros_actuation.md
7. docs/vi/test_checklist.md
8. docs/vi/command_teminal.md
9. docs/vi/ai_workflow/prompt_gate3.md

Mục tiêu hiện tại là bắt đầu và hoàn tất Gate 3: xác nhận NMPC hoạt động đúng trong trạng thái pre-arm có kiểm soát. Gate 3 chưa phải bài test xe chạy và chưa được tự ý arm motor.

Trạng thái đã xác nhận:
- Gate 1: PASS.
- Gate 2 model-fit: PASS tại:
  evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s
- Gate 2 có 184 valid armed transitions, PWM 1580/1585/1590, train RMSE 0.021743 m/s, held-out RMSE 0.007438 m/s, gate <= 0.04 m/s.
- Evidence Gate 2 có rosbag khoảng 59.4 s và 8602 messages.
- Lưu ý review: tên scenario ghi stop2p5 nhưng metadata/processes.txt cho thấy rolling source thực tế dùng stop_duration_s:=1.0, step_duration_s:=6.0, publish_rate_hz:=50.0, max_speed_mps:=0.6. Không được tuyên bố evidence này chứng minh stop 2.5 s.
- Candidate model nằm tại:
  evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s/plots/longitudinal_model/nmpc_model_candidate.yaml
- Candidate có cr3 chạm giới hạn trên của fitter là 20. Candidate chỉ được merge hoặc bật làm model runtime sau khi review rõ ràng; không tự ý ghi đè config mặc định.
- Config mặc định hiện vẫn giữ shadow_mode=true và model_calibrated=false.
- Tần số actuation/control đã được đặt 50 Hz; EKF sensor_timeout=0.15 s; odometry/filtered thực tế khoảng 6.17 Hz.
- Guard tốc độ hiện tại của test rolling là 0.6 m/s.
- /bno055/imu hiện không có publisher; không được tuyên bố hệ thống đang dùng đầy đủ IMU/EKF nếu chưa kiểm tra lại runtime.
- Serial MCU đúng là:
  /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02

Nguyên tắc an toàn bắt buộc:
- Không dùng pkill, killall, ros2 daemon stop để dọn hệ thống khi chưa xác định rõ process; chỉ Ctrl+C process do chính terminal hiện tại khởi động.
- Xe phải ở trạng thái an toàn, E-stop trong tầm tay, motor DISARMED trong toàn bộ Gate 3 pre-arm.
- Phải bắt đầu ghi evidence trước khi chạy test.
- Không gọi /actuation/arm trong bước kiểm tra pre-arm này.
- Không đặt model_calibrated=true trong config mặc định chỉ vì Gate 2 đã PASS.
- Khi lane, odometry, timestamp, solver hoặc control command stale/invalid/NaN, hệ thống phải giữ safe-stop và không được arm.

Trình tự làm việc mong muốn:
1. Kiểm tra source tree, build hiện tại và đọc candidate YAML; nếu phát hiện bất nhất thì báo trước, không tự ý sửa/merge.
2. Tạo một evidence session mới cho Gate 3.
3. Khởi động sensor fusion và các node lane/NMPC theo đúng tài liệu hiện hành.
4. Khởi động micro-ROS actuation bằng serial device đúng ở trên.
5. Xác nhận các điều kiện pre-arm:
   - /line/status có line_detected hợp lệ.
   - /lane/reference có dữ liệu hợp lệ và còn mới.
   - /odometry/filtered có dữ liệu mới.
   - /nmpc/status là ACTIVE_VALID khi NMPC được bật.
   - /control/nmpc chỉ có đúng publisher dự kiến, không có publisher xung đột.
   - /mcu/actuator_cmd chỉ do actuation_manager publish.
   - /actuation/status và /mcu/actuator_state vẫn DISARMED, PWM an toàn 1500/1550.
6. Ghi lại rosbag, log và các kiểm tra topic/service vào session.
7. Đánh giá Gate 3 theo evidence. Chỉ khi tất cả điều kiện trên đạt mới kết luận Gate 3 PASS và chuyển sang chuẩn bị Gate 4 (held active). Nếu thiếu điều kiện nào thì kết luận BLOCKED/FAIL tương ứng và hướng dẫn sửa đúng phạm vi.

Lệnh setup chuẩn:
cd /home/ac/main/1_projects/autonomous_car_hardware_platform_v2
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
source scripts/acados_env.sh

Serial actuation chuẩn:
bash scripts/run_micro_ros_actuation.sh \\
  serial_device:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02

Khi hướng dẫn lệnh, hãy gửi theo từng terminal, ghi rõ terminal nào chạy lâu dài và terminal nào chỉ chạy một lần. Sau mỗi bước phải giải thích output mong đợi và điều kiện dừng an toàn. Không tự ý thực hiện thao tác làm xe chạy.
```

File này là prompt khởi động cho chat Gate 3; `prompt_gate1.md` được giữ lại như tài liệu chuyển tiếp/lịch sử.

## Cập nhật sau Gate 3/Gate 5 — 2026-07-16

Phần prompt phía trên được giữ nguyên để bảo toàn lịch sử khởi động Gate 3.
Trạng thái hiện hành đã thay đổi: Gate 3 active pre-arm PASS, candidate đã
được review/merge với `model.cr3=20.0`, config hardware mặc định dùng
`model_calibrated=true` và `shadow_mode=false` cho test có kiểm soát; Gate 4
held active cũng đã PASS.

Gate 5 test 1, 2 và 3 full map chưa PASS. Test 3 dùng exposure `4000 us`, xe
vượt chuyển làn đơn nhưng safe-stop ở cua phải gắt. Video là
`evidence/test3_gate5.mp4`; quy trình chạy lại được tách tại
[`gate5_manual_test.md`](../gate5_manual_test.md). Hướng tiếp theo là nghiên
cứu image processing dưới ánh sáng thay đổi và ở cua gắt trước khi quy kết
lỗi cho NMPC hoặc tầng khác.

Sau lượt Gate 5 cuối ngày 2026-07-16, bước tiếp theo được chuyển thành
diagnostic image processing + servo-only: motor/ESC phải vô hiệu hóa vật lý,
operator đẩy xe, record ảnh và `cte/kappa/s/heading`, rồi mới quyết định sửa
perception hay NMPC. Prompt cho chat mới nằm tại
[`prompt_gate5_image_diagnostic.md`](prompt_gate5_image_diagnostic.md).
