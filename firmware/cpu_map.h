/*
  cpu_map.h - Header file chọn cấu hình CPU và pin mapping
  
  CHỨC NĂNG:
  - File chọn pin mapping cho các loại processor khác nhau
  - AVR 328p (Arduino Uno) - hỗ trợ chính thức
  - AVR Mega 2560 (Arduino Mega) - do người dùng cung cấp
  - Mỗi processor có file pin mapping riêng (cpu_map_atmega328p.h, etc.)
  
  LƯU Ý: Với processor mới, chỉ cần thêm define name và filename
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
*/

#ifndef cpu_map_h
#define cpu_map_h

// Arduino Uno (ATmega328P) - hỗ trợ chính thức
#ifdef CPU_MAP_ATMEGA328P
  #include "cpu_map/cpu_map_atmega328p.h"
#endif

// Arduino Mega 2560 (ATmega2560) - do người dùng cung cấp
#ifdef CPU_MAP_ATMEGA2560
  #include "cpu_map/cpu_map_atmega2560.h"
#endif

/* 
// Để tùy chỉnh pin map hoặc processor khác:
#ifdef CPU_MAP_CUSTOM_PROC
  // Copy và chỉnh sửa một trong các file cpu_map có sẵn
  // Đảm bảo đổi tên define trong config.h
#endif
*/

#endif
