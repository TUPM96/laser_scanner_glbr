/*
  nuts_bolts.h - Header file định nghĩa, biến, và hàm dùng chung
  
  CHỨC NĂNG:
  - Định nghĩa cơ bản cho GRBL (axis, conversions, macros)
  - Các hàm tiện ích dùng chung
  - Macro và constant quan trọng
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon 
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

#define false 0
#define true 1

// Giá trị index trục - phải bắt đầu từ 0 và liên tục
#define N_AXIS 3 // Số trục
#define X_AXIS 0 // Index trục X
#define Y_AXIS 1 // Index trục Y
#define Z_AXIS 2 // Index trục Z
// #define A_AXIS 3 // Trục A (nếu cần)

// Gán motor CoreXY - KHÔNG THAY ĐỔI
// LƯU Ý: Nếu thay đổi A và B motor axis bindings, sẽ ảnh hưởng đến phương trình CoreXY
#ifdef COREXY
 #define A_MOTOR X_AXIS // Phải là X_AXIS
 #define B_MOTOR Y_AXIS // Phải là Y_AXIS
#endif

// Chuyển đổi đơn vị
#define MM_PER_INCH (25.40)       // mm trong 1 inch
#define INCH_PER_MM (0.0393701)   // inch trong 1 mm
#define TICKS_PER_MICROSECOND (F_CPU/1000000) // Ticks trong 1 micro giây

// Macro tiện ích
#define clear_vector(a) memset(a, 0, sizeof(a))                    // Xóa vector
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS) // Xóa vector float
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define bit(n) (1 << n) 
#define bit_true_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) |= (mask); SREG = sreg; }
#define bit_false_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) &= ~(mask); SREG = sreg; }
#define bit_toggle_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) ^= (mask); SREG = sreg; }
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
void delay_us(uint32_t us);

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
float hypot_f(float x, float y);

#endif
