# Prompt khởi động chat mới — Gate 5 image-processing diagnostic

Bạn đang tiếp quản dự án tại:
`/home/ac/main/1_projects/autonomous_car_hardware_platform_v2`

Hãy đọc trước:

1. `project_state.md`
2. `README.md`
3. `docs/vi/README.md`
4. `docs/vi/operator_runbook.md`
5. `docs/vi/step01_image_processing.md`
6. `docs/vi/step03_nmpc.md`
7. `docs/vi/step04_microros_actuation.md`
8. `docs/vi/test_checklist.md`
9. `docs/vi/gate5_manual_test.md`
10. `docs/vi/ai_workflow/prompt_gate5_image_diagnostic.md`

## Mục tiêu ngày hôm nay

Gate 5 autonomous trước đó chưa PASS. Trước khi chạy lại full-map, hãy thực
hiện một phiên **diagnostic image processing + servo-only** để tách lỗi
perception khỏi lỗi NMPC. Phiên này **không được tuyên bố là Gate 5 PASS**.

Operator sẽ đẩy xe bằng tay qua đoạn thẳng, vào cua và ra cua. Hệ thống ghi
ảnh camera cùng các output của image processing; servo phản ứng theo steering
estimate, còn motor/ESC phải bị vô hiệu hóa.

## Trạng thái đã biết

- Gate 1, Gate 2 model-fit, Gate 3 active pre-arm và Gate 4 held active đã PASS.
- Gate 5 chưa PASS: chưa có ba vòng liên tiếp và các lượt cuối có
  `lane_invalid`, `cte_outside_safe_bound`, `solver_failed`/`ACADOS_MINSTEP`.
- Evidence Gate 2 PASS:
  `evidence/20260714-103708_longitudinal_fit_1580_1585_1590_hold6s_50hz_stop2p5_60s`.
- Evidence Gate 3 PASS:
  `evidence/20260715-095303_gate3_default_model_prearm_20260715`.
- Evidence Gate 4 PASS:
  `evidence/20260716-104744_gate4_held_active_manual_exposure_20260716`.
- Camera diagnostic trước đó:
  `evidence/20260716-123801_gate5_camera_capture_exposure4000_20260716`.
- Lượt Gate 5 cuối:
  `evidence/20260716-125307_gate5_final_rolling_exposure4000_20260716`.
- Camera dùng manual exposure `4000 us`, analogue gain `1.0` là baseline hiện
  tại; không đổi exposure giữa chừng trong một session.
- BNO055 không được sử dụng; fusion chạy với `start_imu:=false`, dùng LiDAR/RF2O.
- Serial MCU chuẩn:
  `/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495365495067184632-if02`
- Giới hạn servo thực tế `1150..1850 us`; tâm `1500 us`; ESC stop `1550 us`.

## Điều kiện an toàn bắt buộc

- Xe ở khu vực an toàn, E-stop trong tầm tay, người giám sát có mặt.
- Trước khi mở actuation, phải xác nhận motor/ESC đã **vô hiệu hóa vật lý**:
  ngắt nguồn động lực hoặc ngắt đường điều khiển ESC theo quy trình phần cứng
  đã thống nhất. Không được chỉ giả định motor tắt vì xe đang được đẩy.
- `actuation_manager` hiện chưa có tham số `servo_only`. Không tự ý thêm
  `servo_only` vào YAML mặc định. `esc_calibrated=false` chỉ làm arm bị từ chối,
  không tạo ra phiên servo-only.
- Nếu chưa có cách vô hiệu hóa motor mà vẫn cho servo hoạt động độc lập, dừng
  và báo blocker; không arm để thử.
- Khi test, `/mcu/actuator_cmd` phải giữ ESC `1550` ở mọi mẫu. Nếu thấy ESC
  khác `1550`, servo chạy ngoài `1150..1850`, nhiều publisher, stale/NaN hoặc
  command invalid: disarm/E-stop ngay và giữ evidence.
- Không dùng `pkill`, `killall`, `ros2 daemon stop`; chỉ Ctrl+C process do chat
  hiện tại khởi động.

## Quy trình thực hiện

1. Kiểm tra source/build, serial, motor disable và không có runtime process
   không rõ chủ sở hữu. Không sửa model NMPC hoặc config mặc định.
2. Khởi động camera/lane với manual exposure `4000 us`, gain `1.0`; fusion với
   `start_imu:=false`; lane adapter. Chỉ khởi động NMPC nếu cần để tạo steering
   estimate và đã có cơ chế chắc chắn drive effort bị ép về `0`.
3. Tạo evidence session **trước khi đẩy xe hoặc arm**.
4. Record tối thiểu các topic:

   `/camera/front_camera/image_raw`, `/line/debug_image`, `/line/fitting_image`,
   `/line/status`, `/line/confidence`, `/line/cte`, `/line/heading_angle`,
   `/line/kappa`, `/line/s_metric`, `/line/kappa_s`, `/lane/reference`,
   `/control/nmpc`, `/nmpc/status`, `/odometry/filtered`, `/scan`,
   `/mcu/actuator_cmd`, `/mcu/actuator_state`, `/mcu/arm`, `/actuation/status`.

5. Trước khi cho servo phản ứng, xác nhận line/lane/reference có timestamp mới,
   các topic dữ liệu có publisher đúng, servo ở `1500`, ESC ở `1550`, và không
   có command motor.
6. Ghi các pha có đánh dấu thời gian trong report/video: đứng yên trên đường
   thẳng, đẩy qua đường thẳng, bắt đầu cua, giữa cua, ra cua, dừng.
7. Sau phiên, disarm, kiểm tra servo `1500` và ESC `1550`, dừng process theo
   thứ tự an toàn, đóng session với `PASS`, `FAIL` hoặc `INCOMPLETE` đúng bằng
   evidence. Phiên diagnostic PASS không đồng nghĩa Gate 5 autonomous PASS.

## Tiêu chí phân tích

- Nếu ảnh/debug image mất biên hoặc nhảy, `line/status` invalid, confidence
  giảm, `cte/kappa/s/heading` nhảy bất hợp lý **và** servo phản ứng sai cùng
  timestamp: ưu tiên sửa image processing/temporal filtering/ROI/fit.
- Nếu ảnh và các output line/lane ổn định, timestamp mới, servo map đúng theo
  steering estimate nhưng NMPC active trước đó vẫn có `ACADOS_MINSTEP` hoặc
  command invalid: ưu tiên phân tích NMPC, reference, odometry và solver.
- Không quy kết nguyên nhân chỉ từ một topic. Phải đối chiếu ảnh, output
  perception, servo PWM và timestamp trong cùng khoảng cua.

## Bàn giao cuối phiên

Report phải ghi rõ: motor disable bằng cách nào, exposure/gain, thời điểm các
pha đẩy xe, số lượng message/ảnh, min/max và độ trễ của `cte/kappa/s/heading`,
servo PWM, ESC PWM, mọi invalid/stale/NaN và kết luận chẩn đoán. Nếu có sửa
image processing, build/unit test và ghi diff/config vào evidence trước khi
chạy lại Gate 5.
