# Archive và tài liệu không canonical

## Phạm vi

`archive/` chứa workspace STM32 cũ, bản PWM thủ công, heartbeat cũ và log
migration. Nó chỉ dùng để tra cứu lịch sử/skeleton; không tham gia build, launch
hoặc flash. Firmware được phép dùng nằm ở
`firmware/stm32/nucleo_f411re_microros`.

`evidence/` cũng không phải archive source: đó là output runtime để chứng minh
một lần test cụ thể. Không sửa report để biến một lần FAIL thành PASS.

## Tài liệu đã rà soát

- Không có `project_state` cũ trong repo.
- Tham chiếu plan bên ngoài trong các README cũ không còn tồn tại ở workspace
  này, nên đã bỏ khỏi đường canonical. Trạng thái hiện tại nằm ở
  [`project_state.md`](../../project_state.md).
- `step01_image_processing.md` và `run_lane_detection.md` đã được cập nhật theo
  ROI `23–46%`, encoding `bgr8`, pipeline HSV/Otsu hiện tại và capture tool mới.
- `image_processing_lane_detection_notes.md` được giữ lại vì còn giá trị nghiên
  cứu, nhưng được đánh dấu rõ là exploratory. Nội dung cyan trong file đó là
  hướng thử nghiệm, không phải màu/threshold đang chạy.

Khi một tài liệu tương lai không còn đúng mà không có giá trị tham khảo, đưa nó
vào archive và thêm entry tại đây; không để hai bản hướng dẫn runtime cùng tồn
tại ở thư mục canonical.
