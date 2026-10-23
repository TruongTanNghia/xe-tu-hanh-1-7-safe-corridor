# Firmware STM32 F411RE cho micro-ROS

Đây là firmware mục tiêu đã kiểm toán cho xe 1/7. Thư mục
`archive/stm32_ws` chỉ là dữ liệu nguồn để đối chiếu và không được flash.

## Giao diện

| Hướng | Topic | Kiểu | Ý nghĩa |
|---|---|---|---|
| Pi → MCU | `/mcu/actuator_cmd` | `std_msgs/Int16MultiArray` | `[servo_us, esc_us]` |
| Pi → MCU | `/mcu/arm` | `std_msgs/Bool` | heartbeat arm; phải được gửi liên tục |
| MCU → Pi | `/mcu/status` | `std_msgs/UInt32` | bộ đếm heartbeat |
| MCU → Pi | `/mcu/actuator_state` | `std_msgs/Int16MultiArray` | `[servo_us, esc_us, flags]` |

Các bit của `flags`: `1=agent connected`, `2=armed`, `4=command fresh`,
`8=arm fresh`, `16=hardware watchdog enabled`.

## Invariant an toàn

- Khởi động luôn ở `servo=1500`, `ESC=1550`, DISARMED.
- Chỉ chạy khi cả `/mcu/arm=true` và `/mcu/actuator_cmd` còn mới trong
  250 ms.
- Mất agent, mất arm hoặc mất command đều trả ngay về stop.
- Servo bị chặn trong `1150..1850 µs`; ESC chỉ cho stop `1550 µs` hoặc
  tiến `1580..1600 µs`; không có reverse.
- Firmware tự tìm lại micro-ROS agent mỗi 500 ms.
- Independent watchdog reset MCU nếu vòng lặp bị treo khoảng 2 giây.

## Nguồn gốc và phần bị loại

Nguồn ban đầu:
`archive/stm32_ws/nucleo_f411re_microros_heartbeat`.
SHA-256 của `Src/main.c` gốc:
`62d9588b74978f8dd4531700cec0a4d0e092c0176b8d3d34b84e279130b21e20`.

Không tái sử dụng các phần sai của bản gốc:

- đường dẫn linker tuyệt đối `/home/phibui2112/...`;
- giới hạn servo `1000..2000` và ESC `1500/1620/1660` không khớp host;
- nhận PWM mà không có arm heartbeat độc lập;
- timeout 500 ms và không reconnect agent;
- comment “neutral-only” mâu thuẫn với code bật ESC.

## Build

Kiểm tra tĩnh trên Pi:

```bash
cd ~/main/1_projects/autonomous_car_hardware_platform_v2
./scripts/verify_firmware_source.sh
```

Build thật:

1. Mở STM32CubeIDE và chọn `File → Import → Existing Projects into Workspace`.
2. Chọn thư mục này; project phải hiện tên `nucleo_f411re_microros`.
3. Chọn cấu hình `Debug`, chạy `Project → Clean`, rồi `Build Project`.
4. Kết quả phải có `0 errors`; lưu ảnh Build Console vào evidence.

Máy Pi hiện không có `arm-none-eabi-gcc`/STM32CubeIDE nên kiểm tra cú pháp
không chứng minh được bước link target. Không flash nếu build target chưa PASS.

## Flash an toàn

Tháo bánh khỏi mặt đất, ngắt nguồn động lực ESC và chỉ cấp nguồn logic trước.
Chỉ flash sau khi các gate host, fusion và NMPC shadow đã PASS. Sau flash:

1. Chạy micro-ROS agent.
2. Xác nhận heartbeat và state vẫn ở `[1500,1550,*]`.
3. Thử mất agent, mất arm, mất command khi bánh còn nhấc.
4. Chỉ sau đó mới cấp nguồn động lực và hiệu chuẩn steering/ESC.
