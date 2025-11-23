/*
  coolant_control.c - Implementation điều khiển làm mát (coolant)
  
  CHỨC NĂNG:
  - Điều khiển bơm làm mát (coolant pump)
  - Hỗ trợ flood (M8) và mist (M7) coolant
  - Quản lý trạng thái bật/tắt coolant
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
*/  

#include "grbl.h"

// Khởi tạo điều khiển coolant
void coolant_init()
{
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
  coolant_stop();
}


void coolant_stop()
{
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
}


void coolant_set_state(uint8_t mode)
{
  if (mode == COOLANT_FLOOD_ENABLE) {
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);

  #ifdef ENABLE_M7  
    } else if (mode == COOLANT_MIST_ENABLE) {
      COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
  #endif

  } else {
    coolant_stop();
  }
}


void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
  coolant_set_state(mode);
}
