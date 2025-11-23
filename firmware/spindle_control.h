/*
  spindle_control.h - Header file điều khiển spindle (trục chính)
  
  CHỨC NĂNG:
  - Điều khiển spindle motor (trục chính)
  - Hỗ trợ PWM để điều khiển tốc độ spindle
  - Quản lý chiều quay (M3/M4) và tốc độ (RPM)
  - Bật/tắt spindle (M5)
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/ 

#ifndef spindle_control_h
#define spindle_control_h 

// Khởi tạo spindle pins và hardware PWM (nếu bật)
void spindle_init();

// Thiết lập chiều quay và tốc độ spindle qua PWM (nếu bật)
void spindle_run(uint8_t direction, float rpm);

// Thiết lập trạng thái và tốc độ spindle
void spindle_set_state(uint8_t state, float rpm);

// Dừng spindle
void spindle_stop();

#endif
