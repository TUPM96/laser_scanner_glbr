/*
  firmware.ino - File chính cho Arduino IDE
  
  CHỨC NĂNG:
  - Wrapper file để Arduino IDE compile GRBL firmware
  - Include grbl.h để load toàn bộ hệ thống
  - Logic chính nằm trong main.c
  
  TÍCH HỢP:
  - I2C communication (i2c_init trong main.c)
  - VL53L0X sensor (vl53l0x_init trong main.c)
  
  SỬ DỤNG:
  - Mở file này trong Arduino IDE
  - Chọn board: Arduino Uno
  - Click Upload

*/

#include "grbl.h"
