/*
  motion_control.h - Header file giao diện cấp cao cho lệnh chuyển động
  
  CHỨC NĂNG:
  - Giao diện cấp cao phát lệnh chuyển động
  - Thực thi chuyển động linear, arc, dwell
  - Thực hiện homing cycle và probe cycle
  - Reset hệ thống
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef motion_control_h
#define motion_control_h

#define HOMING_CYCLE_LINE_NUMBER -1

// Thực thi chuyển động linear trong tọa độ tuyệt đối (mm)
// feed_rate: tốc độ feed (mm/s) hoặc (1/phút)/feed_rate nếu invert_feed_rate = true
#ifdef USE_LINE_NUMBERS
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Thực thi chuyển động arc (offset mode)
// position: tọa độ hiện tại, target: tọa độ đích, offset: offset từ vị trí hiện tại
// axis_XXX: định nghĩa mặt phẳng tròn, axis_linear: hướng helical, radius: bán kính
// is_clockwise_arc: chiều kim đồng hồ
#ifdef USE_LINE_NUMBERS
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc, int32_t line_number);
#else
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);
#endif
  
// Dwell (tạm dừng) trong số giây cụ thể
void mc_dwell(float seconds);

// Thực hiện homing cycle để tìm điểm gốc máy (cần limit switches)
void mc_homing_cycle();

// Thực hiện probe cycle để đo chiều dài tool (cần probe switch)
#ifdef USE_LINE_NUMBERS
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
  uint8_t is_no_error, int32_t line_number);
#else
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
  uint8_t is_no_error);
#endif

// Reset hệ thống - nếu đang chuyển động, dừng tất cả và set alarm
void mc_reset();

#endif
