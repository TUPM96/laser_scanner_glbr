/*
  system.h - Header file cho lệnh cấp hệ thống và xử lý real-time
  
  CHỨC NĂNG:
  - Quản lý trạng thái hệ thống GRBL
  - Xử lý lệnh real-time (status, cycle start/stop, feed hold, etc.)
  - Quản lý alarm và các sự kiện quan trọng
  - Chuyển đổi giữa steps và tọa độ máy
  
  Part of Grbl
  Copyright (c) 2014-2015 Sungeun K. Jeon
*/

#ifndef system_h
#define system_h

#include "grbl.h"

// Bit map lệnh executor - dùng để thông báo thực thi lệnh real-time bất đồng bộ
#define EXEC_STATUS_REPORT  bit(0) // Báo cáo trạng thái (bitmask 00000001)
#define EXEC_CYCLE_START    bit(1) // Bắt đầu chu trình (bitmask 00000010)
#define EXEC_CYCLE_STOP     bit(2) // Dừng chu trình (bitmask 00000100)
#define EXEC_FEED_HOLD      bit(3) // Tạm dừng feed (bitmask 00001000)
#define EXEC_RESET          bit(4) // Reset hệ thống (bitmask 00010000)
#define EXEC_SAFETY_DOOR    bit(5) // Cửa an toàn (bitmask 00100000)
#define EXEC_MOTION_CANCEL  bit(6) // Hủy chuyển động (bitmask 01000000)

// Bit map alarm executor - các alarm hệ thống
#define EXEC_CRITICAL_EVENT     bit(0) // Sự kiện nghiêm trọng (bitmask 00000001)
#define EXEC_ALARM_HARD_LIMIT   bit(1) // Alarm giới hạn cứng (bitmask 00000010)
#define EXEC_ALARM_SOFT_LIMIT   bit(2) // Alarm giới hạn mềm (bitmask 00000100)
#define EXEC_ALARM_ABORT_CYCLE  bit(3) // Alarm hủy chu trình (bitmask 00001000)
#define EXEC_ALARM_PROBE_FAIL   bit(4) // Alarm lỗi đầu dò (bitmask 00010000)
#define EXEC_ALARM_HOMING_FAIL  bit(5) // Alarm lỗi homing (bitmask 00100000)

// Bit map trạng thái hệ thống
#define STATE_IDLE          0      // Trạng thái idle (phải là 0)
#define STATE_ALARM         bit(0) // Trạng thái alarm - khóa tất cả g-code, cho phép truy cập settings
#define STATE_CHECK_MODE    bit(1) // Chế độ kiểm tra g-code - chỉ khóa planner và motion
#define STATE_HOMING        bit(2) // Đang thực hiện homing cycle
#define STATE_CYCLE         bit(3) // Chu trình đang chạy hoặc đang thực thi chuyển động
#define STATE_HOLD          bit(4) // Feed hold đang active
#define STATE_SAFETY_DOOR   bit(5) // Cửa an toàn đang mở - tạm dừng và tắt nguồn
#define STATE_MOTION_CANCEL bit(6) // Hủy chuyển động và quay về idle 

// Define system suspend states.
#define SUSPEND_DISABLE       0      // Must be zero.
#define SUSPEND_ENABLE_HOLD   bit(0) // Enabled. Indicates the cycle is active and currently undergoing a hold.
#define SUSPEND_ENABLE_READY  bit(1) // Ready to resume with a cycle start command.
#define SUSPEND_ENERGIZE      bit(2) // Re-energizes output before resume.
#define SUSPEND_MOTION_CANCEL bit(3) // Cancels resume motion. Used by probing routine.


// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint8_t state;                 // Tracks the current state of Grbl.
  uint8_t suspend;               // System suspend bitflag variable that manages holds, cancels, and safety door.
  uint8_t soft_limit;            // Tracks soft limit errors for the state machine. (boolean)
  
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 
                                 // NOTE: This may need to be a volatile variable, if problems arise.                             

  int32_t probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
  uint8_t probe_succeeded;        // Tracks if last probing cycle was successful.
  uint8_t homing_axis_lock;       // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
} system_t;
extern system_t sys;

volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms.


// Initialize the serial protocol
void system_init();

// Returns if safety door is open or closed, based on pin state.
uint8_t system_check_safety_door_ajar();

// Executes an internal system command, defined as a string starting with a '$'
uint8_t system_execute_line(char *line);

// Execute the startup script lines stored in EEPROM upon initialization
void system_execute_startup(char *line);

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx);

// Updates a machine 'position' array based on the 'step' array sent.
void system_convert_array_steps_to_mpos(float *position, int32_t *steps);

// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps);
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps);
#endif

#endif
