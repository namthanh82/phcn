# Hướng dẫn thiết lập Phần cứng ESP32 + Raspberry Pi + ODrive qua mạng CAN
## Phân bổ chân & Dây nối
1. **Raspberry Pi 3 tới ESP32** (UART 921600 bps)
   * `Pi TX` (thường là GPIO14 / ttyAMA0) -> `ESP32 RX` (Theo code là GPIO3/RX vắt chéo)
   * `Pi RX` (thường là GPIO15 / ttyAMA0) -> `ESP32 TX` (Theo code là GPIO1/TX)
   * `GND` Pi -> `GND` ESP32 (Rất quan trọng!)

2. **ESP32 tới Module MCP2515 qua SPI**
   * Trong trường hợp sử dụng module trung gian có IC MCP2515 + TJA1050 (Board màu xanh cắm chân), kết nối yêu cầu qua giao thức SPI:
   * `ESP32 5V` (Hoặc Vin) -> `VCC` của module MCP2515
   * `ESP32 GND` -> `GND`
   * `ESP32 GPIO_23` (MOSI) -> `SI`
   * `ESP32 GPIO_19` (MISO) -> `SO`
   * `ESP32 GPIO_18` (SCK) -> `SCK`
   * `ESP32 GPIO_5` -> `CS` (Hoặc chân thiết lập ở code `.ino`)
   * *(Lưu ý: Mạch ESP32 phải sử dụng đúng firmware file `esp32_can_bldc_mcp2515.ino` thay vì bản cũ TWAI).*

3. **Cấu hình Trở kháng bus CAN**
   * Đảm bảo trên Bus CAN có đúng 2 điện trở kết cuối `120 Ohm` (Một cái ở đầu: Transceiver ESP32, một cái ở thiết bị cuối cùng (ODrive xa nhất)). Gạt switch `CAN_TERM` trên ODrive cho phù hợp.

## Cấu hình ODrive (Sử dụng `odrivetool`)
Trước khi nối vào mạng CAN, mỗi ODrive cần được cấu hình đúng:
```python
# Ví dụ thiết lập Node ID = 1 cho trục đầu tiên
odrv0.axis0.config.can.node_id = 1
odrv0.axis1.config.can.node_id = 2  # Nếu sử dụng dual axis

# Tốc độ CAN 500kbps (Phải khớp với ESP32)
odrv0.can.config.baud_rate = 500000

# Tự động gửi Feedback Pos/Vel cứ mỗi 10ms (100Hz) hoặc 5ms (200Hz). Điều này giúp ESP32 nhận data ngay mà không cần query.
odrv0.axis0.config.can.encoder_rate_ms = 10 
odrv0.axis1.config.can.encoder_rate_ms = 10 

# Lưu lại
odrv0.save_configuration()
odrv0.reboot()
```

## Giao thức UART PC <-> ESP32 Custom
Để tận dụng tối đa băng thông, giao tiếp UART sử dụng frame nhị phân (Binary) thay vì chữ (ASCII):
* **Request Feedback (Từ Pi gửi ESP32):** `[0xAA] [0xBB] [0x02] [0x00]`
* **Feedback (ESP32 gửi lại Pi):** `[0xAA] [0xBB] [0x02] [Len] [NodeID1] [Pos1_Float] [Vel1_Float] ... [NodeIDn] [PosN_Float] [VelN_Float]`
* **Send Torque (Từ Pi gửi ESP32):** `[0xAA] [0xBB] [0x01] [Len = n*5] [NodeID1] [Torque_Float1] [NodeID2] ...`