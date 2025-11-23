/*
  print.h - Header file hàm format chuỗi đầu ra
  
  CHỨC NĂNG:
  - Format và in chuỗi, số, float ra serial
  - Hỗ trợ in từ Program Memory (PGM)
  - Format đặc biệt cho coordinate, rate, settings values
  - Tool debug để in bộ nhớ còn trống
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef print_h
#define print_h

// In chuỗi từ RAM
void printString(const char *s);

// In chuỗi từ Program Memory (PGM)
void printPgmString(const char *s);

// In số nguyên
void printInteger(long n);

// In số nguyên không dấu 32-bit (base 10)
void print_uint32_base10(uint32_t n);

// In số không dấu 8-bit với base và số chữ số mong muốn
void print_unsigned_int8(uint8_t n, uint8_t base, uint8_t digits); 

// In số không dấu 8-bit (base 2 - nhị phân)
void print_uint8_base2(uint8_t n);

// In số không dấu 8-bit (base 10 - thập phân)
void print_uint8_base10(uint8_t n);

// In số thực với số chữ số thập phân
void printFloat(float n, uint8_t decimal_places);

// Handler in số thực cho các loại biến đặc biệt trong GRBL
// CoordValue: Vị trí hoặc tọa độ (inch hoặc mm)
// RateValue: Feed rate và vận tốc hiện tại (inch/min hoặc mm/min)
// SettingValue: Giá trị settings dấu phẩy động (luôn là mm)
void printFloat_CoordValue(float n);

void printFloat_RateValue(float n);

void printFloat_SettingValue(float n);

// Tool debug: in bộ nhớ còn trống (bytes) - chỉ dùng để debug
void printFreeMemory();

#endif