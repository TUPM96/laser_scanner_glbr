/*
  probe.c - Implementation module probing methods
  
  CHỨC NĂNG:
  - Quản lý probe switch (đầu dò)
  - Giám sát trạng thái probe pin
  - Ghi lại vị trí hệ thống khi probe kích hoạt
  - State machine cho probing cycle
  
  Part of Grbl
  Copyright (c) 2014-2015 Sungeun K. Jeon
*/
  
#include "grbl.h"

// Mask đảo ngược probe pin state tùy theo cài đặt và probing cycle mode
uint8_t probe_invert_mask;


// Khởi tạo probe pin
void probe_init() 
{
  PROBE_DDR &= ~(PROBE_MASK); // Cấu hình làm input pins
  #ifdef DISABLE_PROBE_PIN_PULL_UP
    PROBE_PORT &= ~(PROBE_MASK); // Normal low - cần pull-down ngoài
  #else
    PROBE_PORT |= PROBE_MASK;    // Bật pull-up nội - normal high
  #endif
}

// Cấu hình invert mask cho probe pin
// Set logic pin theo normal-high/normal-low và probing cycle mode (toward/away from workpiece)
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Khởi tạo bằng 0
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}

// Trả về trạng thái probe pin (triggered = true)
uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }

// Giám sát trạng thái probe pin và ghi lại vị trí hệ thống khi phát hiện
// Được gọi bởi stepper ISR mỗi ISR tick
// LƯU Ý: Hàm này phải cực kỳ hiệu quả để không làm chậm stepper ISR
void probe_state_monitor()
{
  if (sys_probe_state == PROBE_ACTIVE) {
    if (probe_get_state()) {
      sys_probe_state = PROBE_OFF;
      memcpy(sys.probe_position, sys.position, sizeof(sys.position));
      bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
    }
  }
}
