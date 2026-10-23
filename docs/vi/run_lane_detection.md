# Hướng dẫn chạy và lưu evidence lane detection

Quy trình này chỉ chạy camera, TF, `image_processing` và node quan sát. Nó không
khởi động controller hoặc actuator.

## 1. Chuẩn bị

1. Đặt xe đứng yên trên track 1/7, để hai biên màu vàng xuất hiện trong camera.
2. Không cấp động lực và không di chuyển xe trong lúc chụp một pose.
3. Build workspace nếu cần:

   ```bash
   bash scripts/build.sh hardware_camera image_processing
   ```

## 2. Chạy pipeline

```bash
bash scripts/run_lane_detection.sh
```

Có thể truyền launch arguments cho camera:

```bash
bash scripts/run_lane_detection.sh \
  manual_controls:=true exposure_time_us:=4000 analogue_gain:=1.0
```

`4000 us` là giá trị thực nghiệm của Gate 5 test 3 trong điều kiện sáng gắt;
không phải giá trị mặc định. Nếu ánh sáng thay đổi, thử từng giá trị trong
session riêng và ghi lại cùng video/rosbag.

Diagnostic kế tiếp cần ghi raw/debug/fitting image cùng `cte`, `kappa`, `s`,
`heading` và servo PWM trong lúc operator đẩy xe qua cua; xem
[`prompt_gate5_image_diagnostic.md`](ai_workflow/prompt_gate5_image_diagnostic.md).

Kết quả mong đợi: camera `640x360`, message `bgr8`, processor nhận CameraInfo,
log có `bands`, `CTE`, `heading`, `kappa`, và không có lỗi thiếu TF. ROI thật
của processor hiện là `40–63%` chiều cao ảnh sau khi camera được đưa ra phía
trước xe; giới hạn ngang mặc định hiện là toàn ảnh `0–100%`.

## 3. Quan sát

Khuyến nghị khi dùng VS Code Remote SSH:

```bash
bash scripts/observe_lane_web.sh
```

Forward port `8080`. Panel hiển thị raw camera, `Processor ROI 40-63%`, debug
scan bands và poly fitting. Viewer desktop vẫn có thể chạy bằng:

```bash
bash scripts/observe_lane.sh
```

Panel crop chỉ là minh họa; dùng `Debug (scan bands)` và `Poly Fitting` làm
evidence chính. Đóng viewer không dừng pipeline ở terminal chạy camera.

### Chế độ web nhẹ

Mặc định `observe_lane_web.sh` chỉ subscribe và hiển thị trang thứ hai
`Tracking / Output`, ở `5 FPS` và JPEG quality `60`. Cách này tránh decode và
encode đồng thời toàn bộ raw/mask trung gian, vốn có thể làm Raspberry Pi 5
đơ khi browser chỉ đang xem trang tracking.

Nếu cần debug lại trang Segmentation trong một phiên riêng:

```bash
LANE_VIEWER_SHOW_SEGMENTATION=true \
LANE_VIEWER_MAX_FPS=3 \
LANE_VIEWER_JPEG_QUALITY=50 \
bash scripts/observe_lane_web.sh
```

Chỉ bật chế độ này khi cần kiểm tra từng mask; không dùng trong lúc xe chạy
active nếu web viewer làm CPU tăng hoặc pipeline có dấu hiệu stale.

## 4. Xem topic

```bash
source /home/ac/ros2_humble/install/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /line/status
ros2 topic echo /line/confidence
ros2 topic echo /line/cte
ros2 topic echo /line/heading_angle
ros2 topic echo /line/kappa
```

Kiểm tra tần số:

```bash
ros2 topic hz /camera/front_camera/image_raw
ros2 topic hz /line/status
```

## 5. Capture evidence hiện tại

Dùng tool tương tác để chụp cùng một snapshot của raw/debug/fitting/panel và
các topic lane:

```bash
bash scripts/capture_lane_pose_evidence.sh \
  --labels straight,enter_curve,exit_curve
```

Nhấn Space cho từng pose theo thứ tự; nhấn `q` để thoát. Có thể chỉ định thư
mục output:

```bash
bash scripts/capture_lane_pose_evidence.sh \
  --labels straight,curve_entry \
  --output-dir evidence/lane_pose_manual
```

Mỗi session có `session.json` và một thư mục cho từng pose, gồm:
`raw.jpg`, `debug.jpg`, `fitting.jpg`, `panel.jpg`, `telemetry.json` và
`echo.txt`. Đọc `project_state.md` để biết session nào là mới nhất và kết luận
được phép rút ra.

`scripts/capture_lane_evidence.sh` vẫn được giữ cho một snapshot đơn lẻ; đây là
compatibility utility, không phải format evidence chính của session mới.

## 6. Tiêu chí quan sát

Một pose chỉ được ghi nhận tốt khi:

- raw camera nhìn thấy cả track và hai biên;
- debug bands chọn vùng lane, không chọn chassis/vật thể ngoài track;
- fitting nằm giữa hai biên và theo đúng đoạn cua;
- status/confidence không mất hoặc nhảy bất thường qua các lần chụp.

Các tiêu chí này chưa thay thế gate calibration, fusion, NMPC và actuation. Không
đổi report thủ công thành PASS và không chạy motor chỉ vì `/line/status` là
`line_detected`.

## 7. Quy tắc process

Không dùng `pkill`, `killall` hoặc dừng process do phiên khác khởi động. Chỉ
nhấn `Ctrl+C` trong đúng terminal của session hiện tại.
