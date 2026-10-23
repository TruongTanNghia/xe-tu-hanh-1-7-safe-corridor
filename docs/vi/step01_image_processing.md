# Bước 1 — image_processing

**Snapshot:** 2026-07-16
**Trạng thái:** build/runtime lane đã có evidence tĩnh sau task “Sửa lỗi xử lý
ảnh đầu tiên”; chưa phải gate tự hành.

## Source và thay đổi hiện tại

Package bắt đầu từ baseline `autonmous_car_research`, branch
`test3-nmpc-validation-cleanup`, commit
`0b0723bb606d24336ed46dacc4a1d716c65886f5`. Working tree hiện đã nâng cấp
baseline này, vì vậy không còn đúng khi nói “thuật toán không thay đổi”.

Các điểm sửa chính:

- camera chuyển message output sang `bgr8`, để khớp với OpenCV/cv_bridge;
- processor tính luminance từ RGB/BGR đúng channel, thay vì dùng một channel
  đầu tiên;
- ROI scan hiện dùng `40–63%` chiều cao ảnh sau khi camera được đưa ra phía
  trước xe, với fallback ngang `25–75%`;
- thêm Otsu trên value đã blur, HSV mask vùng màu mục tiêu `H=15..45`,
  morphology và fit hai boundary khi đủ dữ liệu;
- thêm `boundary_corridor`: khi fit được hai biên vàng hợp lệ, toàn bộ vùng
  giữa hai biên được đưa vào scan mask, không còn phụ thuộc độ sáng từng phía
  của mặt đường; khi không đủ biên vẫn fallback về Otsu cũ;
- thêm phục hồi `partial-boundary`: fit riêng các đoạn còn nhìn thấy của từng
  biên; khi một biên mất tạm thời, dùng độ rộng corridor của frame hợp lệ trước
  đó tối đa `150 ms`; nếu không đủ dữ liệu thì vẫn fallback an toàn;
- không clamp polynomial vượt ngoài ảnh vào mép ảnh; hàng có corridor dự đoán
  không hợp lệ bị bỏ qua để tránh scan pixel đen ngoài lane;
- camera có manual exposure/gain tùy chọn, mặc định vẫn auto;
- web/panel viewer hiển thị cùng ROI `40–63%` với processor.

### Quan sát Gate 5 ngày 2026-07-16

Ba lượt full map chưa đạt Gate 5. Khi giảm manual exposure từ `8000 us` xuống
`4000 us` với analogue gain `1.0`, xe vượt được đoạn chuyển làn đơn và active
lâu hơn, nhưng vẫn phát sinh lane invalid/CTE ngoài bound khi vào cua phải gắt.
Video [`evidence/test3_gate5.mp4`](../../evidence/test3_gate5.mp4) hỗ trợ nhận
định xe chạy sát biên ngoài; rosbag test 3 nằm tại
[`evidence/20260716-115602_gate5_full_map_rolling_exposure4000_20260716`](../../evidence/20260716-115602_gate5_full_map_rolling_exposure4000_20260716).

Đây là dấu hiệu cần nghiên cứu robustness theo ánh sáng, thời gian và hình học
cua; chưa đủ bằng chứng để quy kết lỗi duy nhất cho image processing hoặc NMPC.

### Kế hoạch chẩn đoán tiếp theo

Quan sát operator cho thấy image processing ổn định hơn trên đường thẳng nhưng
có thể suy giảm khi vào cua gắt. Phiên tiếp theo phải tách perception khỏi
NMPC: vô hiệu hóa motor/ESC vật lý, đẩy xe bằng tay, giữ servo là actuator duy
nhất, đồng thời record raw/debug/fitting image và các output `cte`, `kappa`,
`s`, `heading` với timestamp. Chỉ khi dữ liệu perception xấu đồng thời servo
sai mới ưu tiên kết luận lỗi image processing; nếu perception tốt mà
solver/control không ổn định thì chuyển điều tra NMPC.

Lưu ý runtime hiện chưa có `servo_only` parameter trong `actuation_manager`;
không đặt `esc_calibrated=false` để giả lập chế độ này vì nó chặn arm. Phải
ngắt motor/ESC vật lý hoặc bổ sung một test mode riêng có review trước khi
thực hiện.

> Màu/threshold HSV hiện tại là behavior của source (`H=15..45`). Ghi chú về
> cyan trong [`image_processing_lane_detection_notes.md`](image_processing_lane_detection_notes.md)
> chỉ là hướng thử nghiệm, không được dùng để suy ra parameter runtime.

## Runtime contract

- Raspberry Pi 5, Debian 12 arm64, ROS 2 Humble;
- IMX708 Wide, sensor mode `2304x1296`;
- output `640x360`, ROS encoding `bgr8`, nominal `30 FPS`;
- frame camera: `camera_optical_frame`, body frame: `base_link`;
- static TF camera hiện tại: `base_link → camera_optical_frame` với
  `x=0.276 m`, `y=0 m`, `z=0.160 m`; quaternion giữ nguyên theo lần đo trước.
  Đây là cập nhật vị trí cơ khí mới (+13 cm theo X, −4,5 cm theo Z), cần kiểm
  tra lại bằng ảnh/scanband trước khi coi là calibration cuối cùng;
- `scan_top_ratio=0.40`, `scan_bot_ratio=0.63`;
- `scan_left_ratio=0.0`, `scan_right_ratio=1.0` để scanband có thể quan sát
  tới cả mép trái/phải ảnh sau khi camera được đưa ra trước xe;
- `black_threshold=60`, `band_height=6`, `min_valid_bands=6`.
- `boundary_corridor_enabled=true`, `boundary_margin_px=4`.
- `boundary_recovery_enabled=true`, `boundary_recovery_max_age_ms=150`,
  `boundary_recovery_min_observations=3`.
- `boundary_search_left_ratio=0.0`, `boundary_search_right_ratio=1.0` để không
  bỏ qua biên vàng khi nó nằm sát mép ảnh.

`boundary_corridor` và phục hồi biên từng phần đã build thành công ngày
2026-07-17; package test không có lỗi. Chưa coi đây là evidence runtime PASS;
cần replay/ảnh và test xe tiếp theo để xác nhận biên trái bị mất từng đoạn vẫn
cho đủ scanband mà không tạo corridor giả.

### Fix ổn định boundary — bước 1

Ngày 2026-07-27 đã thêm robust boundary fit: mỗi boundary được fit lần đầu,
loại các observation có residual lớn bằng MAD/sigma clipping, rồi fit lại tối
đa hai vòng. Bước này chỉ tác động đến polynomial boundary; chưa thay đổi Otsu,
corridor recovery hoặc temporal smoothing. Cần test cùng một vị trí trước khi
triển khai bước smoothing tiếp theo.

## Evidence mới nhất

Session [`evidence/lane_pose_20260711_154156`](../../evidence/lane_pose_20260711_154156)
đã chụp ba pose tĩnh:

| Pose | `/line/status` | Confidence | CTE |
|---|---|---:|---:|
| straight | `line_detected` | `1.0` | `+0.0178 m` |
| enter_curve | `line_detected` | `1.0` | `-0.0373 m` |
| exit_curve | `line_detected` | `1.0` | `+0.0153 m` |

Ảnh panel cho thấy camera nhìn thấy track, scan bands nằm trong ROI và
polynomial fitting nằm giữa hai biên. Đây là **observed**, chưa gọi là PASS vì
chưa có test lặp theo thời gian, test chuyển động, định lượng false-positive và
đánh giá đầy đủ `/line/heading_error`.

## Build/chạy

```bash
bash scripts/build.sh hardware_camera image_processing
bash scripts/run_lane_detection.sh
bash scripts/observe_lane_web.sh
```

Launch hỗ trợ tùy chọn camera:

```bash
bash scripts/run_lane_detection.sh \
  manual_controls:=true exposure_time_us:=4000 analogue_gain:=1.0 \
  boundary_corridor_enabled:=true boundary_margin_px:=4 \
  boundary_recovery_enabled:=true boundary_recovery_max_age_ms:=150 \
  boundary_recovery_min_observations:=3 \
  boundary_search_left_ratio:=0.0 boundary_search_right_ratio:=1.0 \
  scan_top_ratio:=0.40 scan_bot_ratio:=0.63
```

`4000 us` là giá trị thử nghiệm theo điều kiện sáng của test 3, không thay đổi
camera default. Có thể thử các giá trị lân cận trong session riêng và phải ghi
rõ exposure vào evidence.

Để A/B với fallback Otsu cũ, chạy riêng một session với
`boundary_corridor_enabled:=false`. Không đổi parameter trong lúc xe đang chạy.

Để kiểm tra riêng khả năng phục hồi khi mất một đoạn biên, giữ
`boundary_corridor_enabled:=true` và `boundary_recovery_enabled:=true`. Không
dùng `boundary_recovery_max_age_ms` lớn tùy tiện: thời gian quá dài có thể giữ
độ rộng corridor cũ khi hình học thực tế đã thay đổi.

Không launch controller/actuator từ quy trình này. Trước khi chuyển sang NMPC,
phải chứng minh fitting đúng hai biên trên nhiều pose và giữ lane pipeline trong
shadow.

### Web debug hai trang

Một lệnh duy nhất mở web viewer:

```bash
bash scripts/observe_lane_web.sh
```

Web viewer dùng mặc định cùng ROI `40–63%`. Có thể thử một ROI khác mà không
đổi source bằng:

```bash
LANE_SCAN_TOP_RATIO=0.45 LANE_SCAN_BOTTOM_RATIO=0.68 \
  bash scripts/observe_lane_web.sh
```

Viewer có hai trang Segmentation và Tracking / Output. Các topic debug trung
gian gồm:

```text
/line/debug/yellow_mask
/line/debug/dark_mask
/line/debug/corridor_mask
/line/debug/scan_mask
/line/debug/boundary_overlay
/line/debug/mode
```

Ưu tiên đọc `Final Scan Mask` trước `Scan Bands`: nếu mask cuối đã sai thì lỗi
nằm ở threshold/corridor, chưa cần điều tra polynomial hoặc NMPC.
