/*
  i2c.c - Implementation module giao tiếp I2C (TWI)
  
  CHỨC NĂNG:
  - Khởi tạo và quản lý bus I2C ở tốc độ 100kHz
  - Thực hiện các thao tác start/stop/restart
  - Đọc/ghi dữ liệu 1 byte và nhiều byte
  - Đọc/ghi register của thiết bị I2C
  
  Part of Grbl
  Copyright (c) 2024
*/

#include "i2c.h"
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Khởi tạo bus I2C - Cấu hình tốc độ 100kHz
void i2c_init(void)
{
  TWSR = 0; // Prescaler = 1
  TWBR = ((F_CPU / I2C_SCL_FREQ) - 16) / 2; // Tính TWBR cho 100kHz @ 16MHz
  TWCR = (1 << TWEN); // Enable I2C
}

// Bắt đầu truyền I2C với địa chỉ thiết bị
uint8_t i2c_start(uint8_t address)
{
  uint8_t twst;
  
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Gửi START
  while (!(TWCR & (1 << TWINT))); // Chờ hoàn thành
  
  twst = TW_STATUS & 0xF8;
  if ((twst != TW_START) && (twst != TW_REP_START)) return 1;
  
  TWDR = address; // Gửi địa chỉ thiết bị (8-bit)
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))); // Chờ hoàn thành
  
  twst = TW_STATUS & 0xF8; // Kiểm tra ACK
  if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) return 1;
  
  return 0;
}

// Khởi động lại truyền I2C (dùng khi chuyển từ ghi sang đọc)
uint8_t i2c_restart(uint8_t address)
{
  return i2c_start(address);
}

// Dừng truyền I2C
void i2c_stop(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // Gửi STOP
  while (TWCR & (1 << TWSTO)); // Chờ hoàn thành
}

// Ghi 1 byte dữ liệu qua I2C
uint8_t i2c_write(uint8_t data)
{
  uint8_t twst;
  
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))); // Chờ hoàn thành
  
  twst = TW_STATUS & 0xF8;
  if (twst != TW_MT_DATA_ACK) return 1; // Không có ACK
  
  return 0;
}

// Đọc 1 byte từ I2C và gửi ACK (tiếp tục đọc)
uint8_t i2c_read_ack(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

// Đọc 1 byte từ I2C và gửi NACK (kết thúc đọc)
uint8_t i2c_read_nack(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

// Ghi dữ liệu vào register của thiết bị I2C (1 byte)
uint8_t i2c_write_register(uint8_t address, uint8_t reg, uint8_t data)
{
  uint8_t ret;
  
  ret = i2c_start(address << 1); // Chuyển 7-bit address sang 8-bit
  if (ret) { i2c_stop(); return ret; }
  
  ret = i2c_write(reg); // Ghi địa chỉ register
  if (ret) { i2c_stop(); return ret; }
  
  ret = i2c_write(data); // Ghi dữ liệu
  
  i2c_stop();
  return ret;
}

// Ghi nhiều byte dữ liệu vào register của thiết bị I2C
uint8_t i2c_write_register_multi(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length)
{
  uint8_t i, ret;
  
  ret = i2c_start(address << 1);
  if (ret) { i2c_stop(); return ret; }
  
  ret = i2c_write(reg); // Ghi địa chỉ register
  if (ret) { i2c_stop(); return ret; }
  
  for (i = 0; i < length; i++) { // Ghi từng byte
    ret = i2c_write(data[i]);
    if (ret) { i2c_stop(); return ret; }
  }
  
  i2c_stop();
  return 0;
}

// Đọc dữ liệu từ register của thiết bị I2C (1 byte)
uint8_t i2c_read_register(uint8_t address, uint8_t reg)
{
  uint8_t data;
  
  if (i2c_start(address << 1)) { i2c_stop(); return 0; } // Chế độ ghi
  if (i2c_write(reg)) { i2c_stop(); return 0; } // Ghi địa chỉ register
  if (i2c_restart((address << 1) | 1)) { i2c_stop(); return 0; } // Chuyển sang đọc
  
  data = i2c_read_nack(); // Đọc 1 byte (NACK)
  i2c_stop();
  
  return data;
}

// Đọc nhiều byte dữ liệu từ register của thiết bị I2C
uint8_t i2c_read_register_multi(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length)
{
  uint8_t i;
  
  if (i2c_start(address << 1)) { i2c_stop(); return 1; } // Chế độ ghi
  if (i2c_write(reg)) { i2c_stop(); return 1; } // Ghi địa chỉ register
  if (i2c_restart((address << 1) | 1)) { i2c_stop(); return 1; } // Chuyển sang đọc
  
  for (i = 0; i < length; i++) {
    if (i == (length - 1)) {
      data[i] = i2c_read_nack(); // Byte cuối: NACK
    } else {
      data[i] = i2c_read_ack(); // Các byte đầu: ACK
    }
  }
  
  i2c_stop();
  return 0;
}
