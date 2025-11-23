/*
  grbl.h - File header chính của GRBL
  
  CHỨC NĂNG:
  - File include chính của GRBL firmware
  - Định nghĩa phiên bản và include tất cả module
  - Tích hợp I2C và VL53L0X sensor
  - Quản lý thư viện chuẩn AVR
  
  PHẦN TÍCH HỢP:
  - I2C communication (i2c.h, i2c.c)
  - VL53L0X distance sensor (vl53l0x.h, vl53l0x.c)
  - GRBL modules: serial, stepper, planner, gcode, etc.
*/

#ifndef grbl_h
#define grbl_h

// Phiên bản GRBL
#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD "20160726"

// Thư viện chuẩn AVR
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Include các file module của GRBL
#include "config.h"           // Cấu hình compile-time
#include "nuts_bolts.h"       // Các hàm tiện ích
#include "settings.h"         // Quản lý cài đặt
#include "system.h"           // Hệ thống và trạng thái
#include "defaults.h"         // Giá trị mặc định
#include "cpu_map.h"          // Ánh xạ chân phần cứng
#include "coolant_control.h"  // Điều khiển làm mát
#include "eeprom.h"           // Lưu trữ EEPROM
#include "gcode.h"            // Parser g-code
#include "limits.h"           // Giới hạn và homing
#include "motion_control.h"   // Điều khiển chuyển động
#include "planner.h"          // Lập kế hoạch chuyển động
#include "print.h"            // In ấn và format
#include "probe.h"            // Đầu dò
#include "protocol.h"         // Giao thức giao tiếp
#include "report.h"           // Báo cáo trạng thái
#include "serial.h"           // Giao tiếp serial
#include "spindle_control.h"  // Điều khiển spindle
#include "stepper.h"          // Điều khiển stepper motor
#include "i2c.h"              // Module giao tiếp I2C (tích hợp)
#include "vl53l0x.h"          // Driver sensor VL53L0X (tích hợp)

#endif
