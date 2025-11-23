/*
  probe.h - Header file phương thức probing
  
  CHỨC NĂNG:
  - Quản lý đầu dò (probe) cho đo chiều dài tool
  - State machine cho probing cycle
  - Giám sát trạng thái probe pin
  - Ghi lại vị trí hệ thống khi probe kích hoạt
  
  Part of Grbl
  Copyright (c) 2014-2015 Sungeun K. Jeon
*/
  
#ifndef probe_h
#define probe_h 

// Giá trị định nghĩa state machine cho probing
#define PROBE_OFF     0 // Probing tắt hoặc không sử dụng (phải là 0)
#define PROBE_ACTIVE  1 // Đang theo dõi input pin

// Khởi tạo probe pin
void probe_init();

// Cấu hình mask đảo ngược probe pin
// Dùng để set logic pin theo normal-high/normal-low
// và probing cycle modes (toward/away from workpiece)
void probe_configure_invert_mask(uint8_t is_probe_away);

// Trả về trạng thái probe pin (triggered = true)
// Được gọi bởi g-code parser và probe state monitor
uint8_t probe_get_state();

// Giám sát trạng thái probe pin và ghi lại vị trí hệ thống khi phát hiện
// Được gọi bởi stepper ISR mỗi ISR tick
void probe_state_monitor();

#endif
