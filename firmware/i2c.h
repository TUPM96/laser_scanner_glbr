/*
  i2c.h - Header file cho module giao tiếp I2C (TWI)
  
  CHỨC NĂNG:
  - Cung cấp các hàm giao tiếp I2C cơ bản cho Arduino Uno
  - Hỗ trợ đọc/ghi register của thiết bị I2C (như VL53L0X)
  - Sử dụng TWI hardware trên ATmega328P
  - Tốc độ I2C: 100kHz (Standard Mode)
  
  PHẦN CỨNG:
  - SDA: Pin A4 (PC4)
  - SCL: Pin A5 (PC5)
  
  Part of Grbl
  Copyright (c) 2024
*/

#ifndef i2c_h
#define i2c_h

#include "grbl.h"

// Tốc độ bus I2C - 100kHz ở chế độ chuẩn
#define I2C_SCL_FREQ 100000L

// Khởi tạo bus I2C (TWI - Two Wire Interface)
void i2c_init(void);

// Bắt đầu truyền I2C với địa chỉ thiết bị
// address: địa chỉ I2C của thiết bị (7-bit)
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_start(uint8_t address);

// Khởi động lại truyền I2C (dùng cho đọc sau khi ghi)
// address: địa chỉ I2C của thiết bị
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_restart(uint8_t address);

// Dừng truyền I2C
void i2c_stop(void);

// Ghi 1 byte dữ liệu qua I2C
// data: byte cần ghi
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_write(uint8_t data);

// Đọc 1 byte từ I2C và gửi ACK (tiếp tục đọc)
// Trả về: byte đọc được
uint8_t i2c_read_ack(void);

// Đọc 1 byte từ I2C và gửi NACK (kết thúc đọc)
// Trả về: byte đọc được
uint8_t i2c_read_nack(void);

// Ghi dữ liệu vào register của thiết bị I2C
// address: địa chỉ I2C của thiết bị (7-bit)
// reg: địa chỉ register cần ghi
// data: dữ liệu cần ghi (1 byte)
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_write_register(uint8_t address, uint8_t reg, uint8_t data);

// Ghi nhiều byte dữ liệu vào register của thiết bị I2C
// address: địa chỉ I2C của thiết bị (7-bit)
// reg: địa chỉ register cần ghi
// data: con trỏ đến mảng dữ liệu cần ghi
// length: số byte cần ghi
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_write_register_multi(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length);

// Đọc dữ liệu từ register của thiết bị I2C
// address: địa chỉ I2C của thiết bị (7-bit)
// reg: địa chỉ register cần đọc
// Trả về: giá trị đọc được (1 byte), 0 nếu lỗi
uint8_t i2c_read_register(uint8_t address, uint8_t reg);

// Đọc nhiều byte dữ liệu từ register của thiết bị I2C
// address: địa chỉ I2C của thiết bị (7-bit)
// reg: địa chỉ register cần đọc
// data: con trỏ đến mảng lưu dữ liệu đọc được
// length: số byte cần đọc
// Trả về: 0 nếu thành công, 1 nếu lỗi
uint8_t i2c_read_register_multi(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length);

#endif
