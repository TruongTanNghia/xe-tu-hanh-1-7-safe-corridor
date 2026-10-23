# Ghi chú thử nghiệm xử lý ảnh cho lane detection

> **Không canonical.** Trạng thái và parameter đang chạy nằm ở
> [`project_state.md`](../../project_state.md) và source. File này giữ lại các
> hướng nghiên cứu từ cuộc hội thoại “Sửa lỗi xử lý ảnh đầu tiên”; không dùng nó
> để suy ra màu track, ROI hoặc threshold runtime.

Pipeline hiện tại đã khác phần mô tả cũ: source dùng ảnh `bgr8`, luminance đúng
channel, Otsu/value mask và HSV `H=15..45` để xử lý biên màu vàng. Phần cyan bên
dưới chỉ là phương án thay thế nếu evidence tương lai chứng minh màu track khác.

## Quan sát Gate 5 — 2026-07-16

Test 3 với manual exposure `4000 us` vượt chuyển làn đơn nhưng chưa giữ được
lane khi vào cua phải gắt và sau đó safe-stop. So với lượt `8000 us`, kết quả
đầu đoạn tốt hơn; đây là tín hiệu để nghiên cứu exposure/threshold thích ứng,
temporal tracking và xử lý hình học cua. Đây chỉ là hướng điều tra, không phải
kết luận image processing đã được chứng minh là nguyên nhân duy nhất.

## Diagnostic được thống nhất cho lượt kế tiếp

Không chạy autonomous full map ngay. Dùng một phiên servo-only có motor/ESC đã
vô hiệu hóa vật lý, đẩy xe bằng tay qua đường thẳng và cua. Ghi đồng bộ ảnh
raw/debug/fitting với `cte`, `kappa`, `s`, `heading`, confidence, line status,
lane reference và servo PWM. Đánh giá theo timestamp tại đoạn bắt đầu cua,
giữa cua và ra cua. Đây là cách kiểm tra giả thuyết perception mà không để lỗi
NMPC hoặc lực kéo motor che khuất nguyên nhân.

Trong code hiện tại `actuation_manager` chưa có `servo_only`; không dùng
`esc_calibrated=false` như một workaround vì nó làm arm bị từ chối. ESC phải
được xác nhận giữ `1550` suốt phiên.

## Boundary corridor — bản sửa 2026-07-17

Ảnh từ `evidence/20260717_mannual_test` xác nhận Otsu toàn ROI làm mất một
phần scanband khi cùng mặt đường có vùng đen và vùng chá sáng. Source hiện đã
thêm `boundary_corridor_enabled=true`: nếu hai biên vàng được fit đủ tốt,
processor tạo mask đen liên tục giữa hai biên, với margin mặc định `4 px`.

Nếu không đủ hai biên, processor không tự đoán toàn lane mà quay về fallback
Otsu cũ. Đây là bản đã build, nhưng chưa được xác nhận bằng test runtime mới;
cần kiểm tra `bands`, debug image, CTE, heading và kappa trong điều kiện chá
sáng trước khi dùng làm bằng chứng Gate 5.

## Partial-boundary recovery — bản sửa 2026-07-17

Ảnh `evidence/20260717_mannual_test/image copy 5.png` cho thấy một biên có thể
chỉ còn xuất hiện ở một đoạn của ROI khi xe vào cua. Pipeline đã được mở rộng
để không yêu cầu hai biên phải xuất hiện trên cùng mọi scanband:

1. Các run vàng được thu thập và fit riêng cho biên trái/phải.
2. Nếu một biên mất tạm thời nhưng biên còn lại vẫn đủ tối thiểu 3 observation,
   dùng độ rộng corridor của frame đầy đủ gần nhất để suy ra biên bị mất.
3. Prior chỉ có hiệu lực tối đa `150 ms`; hết thời gian hoặc corridor không hợp
   lệ thì không lấy mép ảnh làm biên giả và để pipeline fallback/safe-stop.

Parameter runtime:

```text
boundary_recovery_enabled=true
boundary_recovery_max_age_ms=150
boundary_recovery_min_observations=3
boundary_search_left_ratio=0.0
boundary_search_right_ratio=1.0
```

Vùng tìm biên mặc định nay bao phủ toàn bộ chiều rộng ảnh. Điều này cần thiết
cho trường hợp biên trái/phải nằm sát mép camera khi xe vào cua; bản cũ bỏ qua
5% hai mép ảnh nên có thể loại mất chính boundary. Vẫn giữ kiểm tra khoảng
cách tối thiểu giữa hai biên và không dùng mép ảnh làm boundary nếu không có
run màu vàng hợp lệ.

Corridor cũng không còn clamp polynomial extrapolation vượt ngoài ảnh vào mép
trái/phải. Hàng nào dự đoán hình học vượt ngoài ảnh sẽ bị bỏ qua, tránh để
`scan_row()` lấy pixel đen ngoài lane. Đây là thay đổi an toàn sau khi quan sát
`image copy 7.png` và `image copy 8.png`.

Đây là bản build PASS, chưa phải runtime PASS. Cần kiểm tra frame vào cua trên
panel và xác nhận `bands`, CTE, heading, kappa không bị nhảy khi một biên mất
một đoạn. Nếu biên vàng bị mất hoàn toàn lâu hơn thời gian prior, safe-stop vẫn
là hành vi đúng.

## Web debug hai trang — 2026-07-17

Web viewer hiện có một lệnh khởi động nhưng hai trang chuyển qua nút:

- **Segmentation:** raw, ROI, yellow mask, Otsu dark mask, corridor mask và
  final scan mask;
- **Tracking / Output:** boundary geometry, scan bands, poly fitting và metrics
  với `mode`, status, confidence, CTE, heading, kappa.

Các stage image được publish trên namespace `/line/debug/*`. Khi debug, đọc
`Final Scan Mask` trước `Scan Bands`; đây là cách phân biệt lỗi segmentation
với lỗi fitting.

## Boundary stability — bước 1, 2026-07-27

Đã thêm robust fit cho từng boundary bằng hai vòng MAD/sigma clipping. Mục tiêu
là loại các run vàng lệch do phản xạ trước khi polynomial kéo méo corridor.
Chưa thêm temporal smoothing; cần so sánh nhiều frame ở cùng một pose trước khi
quyết định bước tiếp theo.

## Cyan là màu gì?

Cyan là màu xanh lơ / xanh ngọc, nằm giữa xanh lá và xanh dương.

Trong RGB, cyan lý tưởng thường gần:

```text
RGB(0, 255, 255)
```

Trong OpenCV, ảnh màu thường dùng thứ tự BGR, nên cyan lý tưởng là:

```text
BGR(255, 255, 0)
```

Trong HSV của OpenCV, cyan thường nằm xấp xỉ:

```text
H: 75 - 105
S: 50 - 255
V: 40 - 255
```

Ý tưởng của HSV cyan mask là không hỏi “pixel này tối không?”, mà hỏi “pixel này có giống màu biên cyan không?”. Vì HSV tách màu khỏi độ sáng tương đối tốt hơn RGB/BGR, nó thường ổn định hơn khi ánh sáng môi trường thay đổi.

Ví dụ logic mask:

```cpp
cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

cv::inRange(
    hsv,
    cv::Scalar(75, 50, 40),
    cv::Scalar(105, 255, 255),
    cyan_mask
);

cv::morphologyEx(cyan_mask, cyan_mask, cv::MORPH_CLOSE, kernel);
cv::morphologyEx(cyan_mask, cyan_mask, cv::MORPH_OPEN, small_kernel);
```

## Hướng 1: Segment giống pipeline hiện tại

Đây là hướng “tìm vùng đường / vùng lane bằng threshold rồi fit đường giữa”. Nó gần với cách code hiện tại đang làm: scan từng row, tìm vùng/tâm, fit polynomial.

Pipeline chuẩn hơn:

```text
camera image
→ undistort / crop ROI
→ convert RGB/BGR sang HSV hoặc Lab
→ illumination normalization
→ adaptive threshold / color threshold
→ morphology
→ connected components
→ reject outlier
→ fit polynomial / spline
→ temporal tracking
→ confidence
```

Các cải tiến nên ưu tiên nếu vẫn giữ kiến trúc hiện tại:

- Không dùng brightness cố định kiểu `black_threshold=60` cho mọi điều kiện ánh sáng.
- Dùng HSV/Lab hoặc grayscale luminance đúng cách, thay vì lấy một channel đầu tiên của ảnh.
- Dùng adaptive threshold theo từng vùng ảnh hoặc từng band.
- Dùng morphology để nối vùng bị đứt và bỏ đốm nhiễu nhỏ.
- Reject outlier trước khi fit polynomial.
- Confidence không chỉ dựa vào số band valid, mà thêm độ liên tục, độ cong hợp lý, residual fitting, độ rộng lane và độ gần với frame trước.

Keyword nên đọc:

- `HSV color thresholding lane detection`
- `adaptive threshold lane detection`
- `illumination invariant road detection`
- `shadow removal road segmentation`
- `Retinex lane detection`
- `CLAHE lane detection`
- `morphological filtering lane detection`
- `connected component lane segmentation`
- `RANSAC polynomial lane fitting`
- `Kalman filter lane tracking`
- `inverse perspective mapping lane detection`

Nguồn/paper gợi ý:

- [`zhou2021.pdf`](zhou2021.pdf) — bản local của paper về adaptive threshold
  theo điều kiện ánh sáng; dùng để nghiên cứu, không phải dependency runtime.

- Road Lane Detection Robust to Shadows Based on a Fuzzy System  
  https://www.mdpi.com/1424-8220/17/11/2475

- Real-Time Adaptive Threshold Adjustment for Lane Detection Application under Different Lighting Conditions using Model-Free Control  
  https://www.researchgate.net/publication/357052972_Real-Time_Adaptive_Threshold_Adjustment_for_Lane_Detection_Application_under_Different_Lighting_Conditions_using_Model-Free_Control

- Eliminating Uneven Illumination for Accurate Lane Line Detection: A Dark Channel Prior Algorithm Approach  
  https://saemobilus.sae.org/papers/eliminating-uneven-illumination-accurate-lane-line-detection-a-dark-channel-prior-algorithm-approach-2023-01-5039

- Lane Detection System for Driver Assistance in Vehicles  
  https://arxiv.org/pdf/2410.04046

## Hướng 2: Quan sát trực tiếp hai biên đường

Đây là hướng phát hiện trực tiếp left boundary và right boundary, sau đó lấy centerline làm reference cho controller.

Pipeline:

```text
camera image
→ crop ROI
→ HSV cyan mask
→ morphology
→ perspective transform / bird-eye view nếu có thể
→ tìm component trái/phải
→ fit left boundary
→ fit right boundary
→ enforce lane width ≈ 0.90 m hoặc pixel-width tương ứng
→ centerline = average(left, right)
→ temporal filter / Kalman
```

Ưu điểm:

- Ít bị nhầm bóng tối là đường.
- Ít bị vùng đen/chói làm sai tâm.
- Có ràng buộc vật lý: hai biên phải tương đối song song và cách nhau gần cố định.
- Nếu mất một biên ngắn hạn, vẫn có thể suy ra centerline từ biên còn lại cộng/trừ lane width.

Keyword nên đọc:

- `lane boundary detection`
- `dual lane boundary tracking`
- `parallel lane constraint`
- `inverse perspective mapping lane boundary`
- `sliding window lane boundary detection`
- `RANSAC lane boundary fitting`
- `Kalman filter lane tracking`
- `B-Snake lane detection`
- `active contour lane boundary detection`
- `row-based lane detection`
- `structure-aware lane detection`

Nguồn/paper gợi ý:

- Robust Lane Detection and Tracking with RANSAC and Kalman Filter  
  https://kth.diva-portal.org/smash/record.jsf?pid=diva2%3A482500

- Lane Detection and Tracking using B-Snake  
  https://www.sciencedirect.com/science/article/abs/pii/S0262885603002105

- Lane Detection and Tracking Using a Parallel-snake Approach  
  https://link.springer.com/article/10.1007/s10846-014-0075-0

- Ultra Fast Structure-aware Deep Lane Detection  
  https://arxiv.org/abs/2004.11757

- Spatial CNN for Traffic Scene Understanding  
  https://ojs.aaai.org/index.php/AAAI/article/view/12301

- LaneNet: Real-Time Lane Detection Networks for Autonomous Driving  
  https://arxiv.org/abs/1807.01726

## Khuyến nghị ngắn hạn cho xe hiện tại

Nếu cần cải thiện nhanh nhưng ít thay đổi kiến trúc, chọn hướng 1 trước:

```text
raw image
→ grayscale/luminance đúng cách
→ median blur 3x3
→ CLAHE hoặc adaptive threshold
→ morphology close/open
→ scan row/band như hiện tại
→ fit cubic như hiện tại
```

Nếu hướng 1 vẫn bị fail do ánh sáng/chói, chuyển dần sang hướng 2:

```text
HSV cyan mask
→ detect hai biên
→ lấy midpoint/centerline
→ feed cho NMPC
```
