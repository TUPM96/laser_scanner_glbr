/*
  protocol.h - Header file giao thức thực thi GRBL
  
  CHỨC NĂNG:
  - Điều khiển giao thức thực thi và thủ tục của GRBL
  - Xử lý giao tiếp serial và thực thi g-code
  - Quản lý vòng lặp chính và real-time commands
  - Xử lý cycle start/stop, feed hold, buffer synchronization
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef protocol_h
#define protocol_h

// Kích thước buffer dòng từ serial input stream
// LƯU Ý: Chuẩn g-code hỗ trợ đến 256 ký tự, nhưng hiện tại dùng 80
// Có thể tăng trong tương lai nếu có đủ bộ nhớ
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// Bắt đầu vòng lặp chính GRBL - xử lý tất cả ký tự từ serial và thực thi
void protocol_main_loop();

// Kiểm tra và thực thi lệnh real-time tại các điểm dừng trong chương trình chính
void protocol_execute_realtime();

// Notify the stepper subsystem to start executing the g-code program in buffer.
// void protocol_cycle_start();

// Reinitializes the buffer after a feed hold for a resume.
// void protocol_cycle_reinitialize(); 

// Initiates a feed hold of the running program
// void protocol_feed_hold();

// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start();

// Block until all buffered steps are executed
void protocol_buffer_synchronize();

#endif
