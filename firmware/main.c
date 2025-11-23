/*
  main.c - File chính của GRBL firmware
  
  CHỨC NĂNG:
  - Khởi tạo tất cả các module hệ thống
  - Quản lý vòng lặp chính của GRBL
  - Xử lý reset và re-initialization
  - Tích hợp I2C và VL53L0X sensor
  
  PHẦN KHỞI TẠO:
  - Serial communication
  - Settings từ EEPROM
  - I2C bus cho VL53L0X
  - Stepper motors
  - System pins và interrupts
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#include "grbl.h"

// Biến toàn cục hệ thống
system_t sys; 


int main(void)
{
  // Khởi tạo hệ thống khi bật nguồn.
  serial_init();   // Thiết lập tốc độ baud serial và ngắt
  settings_init(); // Tải cài đặt Grbl từ EEPROM
  i2c_init();      // Khởi tạo bus I2C cho sensor VL53L0X
  stepper_init();  // Cấu hình chân stepper và bộ hẹn giờ ngắt
  system_init();   // Cấu hình chân và ngắt thay đổi chân
  
  memset(&sys, 0, sizeof(system_t));
  sys.abort = true;
  sei(); // Bật ngắt
  
  // Thiết lập alarm nếu homing được bật
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif
  
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif
  
  // Vòng lặp chính: khởi tạo và xử lý reset
  for(;;) {
    // Reset các hệ thống
    serial_reset_read_buffer();
    gc_init();
    spindle_init();
    coolant_init();
    limits_init(); 
    probe_init();
    // Khởi tạo sensor (giống code mẫu: sensor.init())
    if (vl53l0x_init()) {
      // Cấu hình sensor (giống code mẫu: sensor.setTimeout(500))
      vl53l0x_setTimeout(500);
      // Khởi động continuous mode với period 100ms (giống code mẫu: sensor.startContinuous(100))
      vl53l0x_startContinuous(100);
    }
    plan_reset();
    st_reset();

    // Đồng bộ vị trí
    plan_sync_position();
    gc_sync_position();

    // Reset biến hệ thống
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
    sys.soft_limit = false;
              
    // Vòng lặp chính: xử lý g-code và thực thi
    protocol_main_loop();
  }
  return 0;
}
