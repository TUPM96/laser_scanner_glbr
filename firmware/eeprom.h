/*
  eeprom.h - Header file phương thức EEPROM
  
  CHỨC NĂNG:
  - Đọc/ghi dữ liệu từ EEPROM
  - Copy dữ liệu với checksum để đảm bảo tính toàn vẹn
  - Lưu trữ cài đặt và dữ liệu quan trọng
  
  Part of Grbl
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#ifndef eeprom_h
#define eeprom_h

// Đọc 1 byte từ EEPROM tại địa chỉ
unsigned char eeprom_get_char(unsigned int addr);

// Ghi 1 byte vào EEPROM tại địa chỉ
void eeprom_put_char(unsigned int addr, unsigned char new_value);

// Copy dữ liệu vào EEPROM với checksum
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);

// Copy dữ liệu từ EEPROM với checksum (trả về 0 nếu checksum hợp lệ)
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);

#endif
