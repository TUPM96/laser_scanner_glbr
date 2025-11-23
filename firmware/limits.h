/*
  limits.h - Header file giới hạn và homing cycle
  
  CHỨC NĂNG:
  - Quản lý limit switches (giới hạn cứng)
  - Thực hiện homing cycle (tìm điểm gốc)
  - Kiểm tra soft limits (giới hạn mềm)
  - Xử lý hard limit interrupts
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon  
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef limits_h
#define limits_h 

// Khởi tạo module limits
void limits_init();

// Tắt hard limits
void limits_disable();

// Trả về trạng thái limits dưới dạng bit-wise uint8
uint8_t limits_get_state();

// Thực hiện một phần của homing cycle dựa trên cài đặt
void limits_go_home(uint8_t cycle_mask);

// Kiểm tra vi phạm soft limits
void limits_soft_check(float *target);

#endif