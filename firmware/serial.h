/*
  serial.h - Header file module giao tiếp serial
  
  CHỨC NĂNG:
  - Các hàm cấp thấp gửi/nhận byte qua cổng serial
  - Quản lý buffer TX và RX
  - Hỗ trợ XON/XOFF flow control
  - Interrupt-driven communication
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef serial_h
#define serial_h

// Kích thước buffer
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128  // Buffer đọc
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64   // Buffer ghi
#endif

#define SERIAL_NO_DATA 0xff  // Không có dữ liệu

// Cấu hình XON/XOFF flow control (nếu bật)
#ifdef ENABLE_XONXOFF
  #define RX_BUFFER_FULL 96  // Ngưỡng cao XOFF
  #define RX_BUFFER_LOW 64   // Ngưỡng thấp XON
  #define SEND_XOFF 1
  #define SEND_XON 2
  #define XOFF_SENT 3
  #define XON_SENT 4
  #define XOFF_CHAR 0x13
  #define XON_CHAR 0x11
#endif

// Khởi tạo giao tiếp serial
void serial_init();

// Ghi 1 byte vào buffer TX
void serial_write(uint8_t data);

// Đọc 1 byte từ buffer RX
uint8_t serial_read();

// Reset và xóa buffer RX (dùng cho e-stop và reset)
void serial_reset_read_buffer();

// Trả về số byte đã dùng trong buffer RX
uint8_t serial_get_rx_buffer_count();

// Trả về số byte đã dùng trong buffer TX (chủ yếu dùng để debug)
uint8_t serial_get_tx_buffer_count();

#endif
