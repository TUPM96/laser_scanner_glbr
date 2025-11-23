/*
  coolant_control.h - Header file điều khiển làm mát (coolant)
  
  CHỨC NĂNG:
  - Điều khiển bơm làm mát (coolant pump)
  - Hỗ trợ flood (M8) và mist (M7) coolant
  - Quản lý trạng thái bật/tắt coolant
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
*/

#ifndef coolant_control_h
#define coolant_control_h 

// Khởi tạo điều khiển coolant
void coolant_init();

// Dừng tất cả coolant
void coolant_stop();

// Thiết lập trạng thái coolant (mode)
void coolant_set_state(uint8_t mode);

// Chạy coolant với mode cụ thể
void coolant_run(uint8_t mode);

#endif