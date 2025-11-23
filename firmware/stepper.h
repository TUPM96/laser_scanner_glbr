/*
  stepper.h - Header file driver stepper motor
  
  CHỨC NĂNG:
  - Thực thi kế hoạch chuyển động từ planner.c bằng stepper motors
  - Quản lý interrupt timer để tạo xung step
  - Xử lý acceleration và deceleration
  - Điều khiển enable/disable stepper motors
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef stepper_h
#define stepper_h 

// Kích thước buffer segment (mặc định 6)
#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Khởi tạo và cấu hình hệ thống stepper motor
void stepper_init();

// Bật stepper motors (chưa bắt đầu chu trình)
void st_wake_up();

// Tắt stepper motors ngay lập tức
void st_go_idle();

// Tạo mask đảo ngược cho step và direction ports
void st_generate_step_dir_invert_masks();

// Reset biến hệ thống stepper
void st_reset();
             
// Nạp lại buffer step segment (gọi liên tục bởi hệ thống real-time)
void st_prep_buffer();

// Cập nhật tham số block đang thực thi khi plan thay đổi
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef REPORT_REALTIME_RATE
float st_get_realtime_rate();
#endif

#endif
