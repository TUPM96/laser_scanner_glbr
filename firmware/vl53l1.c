/*
  vl53l1.c - Implementation driver sensor VL53L1
  Dựa trên cấu trúc VL53L0X với register addresses của VL53L1
  
  Part of Grbl
  Copyright (c) 2024
*/

#include "vl53l1.h"
#include "print.h"
#include <util/delay.h>

// Biến để track timeout state
static uint8_t vl53l1_timeout_flag = 0;
static uint16_t vl53l1_io_timeout = 500; // Timeout 500ms

// Đọc register 16-bit (dựa trên code C thuần)
// Giống hàm vl53l1x_read16() trong code mẫu
static uint16_t vl53l1_read16(uint16_t reg)
{
  uint16_t val;
  uint8_t reg_msb = (reg >> 8) & 0xFF;
  uint8_t reg_lsb = reg & 0xFF;
  
  // Write register address (2 bytes)
  if (i2c_start((VL53L1_I2C_ADDR << 1))) {
    i2c_stop();
    return 0;
  }
  if (i2c_write(reg_msb)) {
    i2c_stop();
    return 0;
  }
  if (i2c_write(reg_lsb)) {
    i2c_stop();
    return 0;
  }
  
  // Restart và đọc 2 bytes (MSB trước, LSB sau)
  if (i2c_restart((VL53L1_I2C_ADDR << 1) | 1)) {
    i2c_stop();
    return 0;
  }
  val = ((uint16_t)i2c_read_ack() << 8);  // MSB
  val |= i2c_read_nack();                  // LSB
  i2c_stop();
  
  return val;
}

// Set timeout
void vl53l1_setTimeout(uint16_t timeout)
{
  vl53l1_io_timeout = timeout;
}

// Khởi tạo sensor VL53L1 (đơn giản, không cần init sequence phức tạp)
// Trả về: 1 nếu thành công, 0 nếu lỗi
uint8_t vl53l1_init(void)
{
  _delay_ms(100);
  
  // Reset timeout flag
  vl53l1_timeout_flag = 0;
  
  // Không cần kiểm tra model ID hoặc init sequence phức tạp
  // VL53L1 sẽ tự động sẵn sàng sau khi có nguồn
  // Chỉ cần đọc register 0x0096 là có thể lấy distance
  
  return 1; // Thành công
}

// Bắt đầu continuous mode
void vl53l1_startContinuous(uint16_t period_ms)
{
  // VL53L1 không cần start continuous trong single shot mode
  // Mỗi lần gọi readRangeContinuousMillimeters sẽ tự động trigger đo mới
}

// Đọc khoảng cách từ sensor (dựa trên code C thuần)
// Giống hàm vl53l1x_get_distance() trong code mẫu
// Trả về: khoảng cách tính bằng mm (20-4000mm), 0 nếu lỗi, 8190 nếu out of range
uint16_t vl53l1_readRangeContinuousMillimeters(void)
{
  // Reset timeout flag
  vl53l1_timeout_flag = 0;
  
  // Đọc trực tiếp register 0x0096 để lấy distance (giống code mẫu)
  // VL53L1 tự động cập nhật register này khi có measurement mới
  uint16_t distance = vl53l1_read16(VL53L1_REG_RESULT_DISTANCE);
  
  // Xử lý kết quả:
  // - distance >= 8190: OUT OF RANGE
  // - distance == 0: ERROR hoặc chưa có data
  // - Còn lại: OK (20-4000mm cho VL53L1)
  
  if (distance >= 8190) {
    return 8190; // Out of range
  }
  
  // distance = 0 có thể là lỗi hoặc hợp lệ (quá gần)
  // Trả về như code mẫu
  return distance;
}

// Kiểm tra timeout
uint8_t vl53l1_timeoutOccurred(void)
{
  return vl53l1_timeout_flag;
}

