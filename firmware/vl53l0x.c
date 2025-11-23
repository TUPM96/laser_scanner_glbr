/*
  vl53l0x.c - Implementation driver sensor VL53L0X
  Dựa trên code C thuần từ VL53L0X_Basic_C.ino
  
  Part of Grbl
  Copyright (c) 2024
*/

#include "vl53l0x.h"
#include "print.h"
#include <util/delay.h>

// Biến để track timeout state
static uint8_t vl53l0x_timeout_flag = 0;
static uint16_t vl53l0x_io_timeout = 500; // Timeout 500ms

// Ghi register (sử dụng I2C functions có sẵn)
static void vl53_write_reg(uint8_t reg, uint8_t value)
{
  i2c_write_register(VL53L0X_I2C_ADDR, reg, value);
}

// Đọc register (sử dụng I2C functions có sẵn)
static uint8_t vl53_read_reg(uint8_t reg)
{
  return i2c_read_register(VL53L0X_I2C_ADDR, reg);
}

// Đọc nhiều register (sử dụng I2C functions có sẵn)
static void vl53_read_multi(uint8_t reg, uint8_t* buffer, uint8_t len)
{
  i2c_read_register_multi(VL53L0X_I2C_ADDR, reg, buffer, len);
}

// Set timeout (giống sensor.setTimeout(500))
void vl53l0x_setTimeout(uint16_t timeout)
{
  vl53l0x_io_timeout = timeout;
}

// Khởi tạo sensor VL53L0X (dựa trên code C thuần)
// Trả về: 1 nếu thành công, 0 nếu lỗi
uint8_t vl53l0x_init(void)
{
  _delay_ms(100);
  
  uint8_t model_id = vl53_read_reg(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  if (model_id != 0xEE) {
    return 0; // Sensor không tồn tại
  }
  
  // Basic init sequence (giống code C thuần)
  vl53_write_reg(0x88, 0x00);
  vl53_write_reg(0x80, 0x01);
  vl53_write_reg(0xFF, 0x01);
  vl53_write_reg(0x00, 0x00);
  vl53_read_reg(0x91);
  vl53_write_reg(0x00, 0x01);
  vl53_write_reg(0xFF, 0x00);
  vl53_write_reg(0x80, 0x00);
  
  // Reset timeout flag
  vl53l0x_timeout_flag = 0;
  
  return 1; // Thành công
}

// Bắt đầu continuous mode (không cần trong single shot mode, nhưng giữ API)
void vl53l0x_startContinuous(uint16_t period_ms)
{
  // Không cần làm gì trong single shot mode
  // Mỗi lần gọi readRangeContinuousMillimeters sẽ tự động trigger đo mới
}

// Đọc khoảng cách từ sensor (dựa trên code C thuần)
// Giống code mẫu: uint16_t distance = sensor.readRangeContinuousMillimeters();
// Trả về: khoảng cách tính bằng mm (20-2000mm), 0 nếu lỗi, 8190 nếu out of range
uint16_t vl53l0x_readRangeContinuousMillimeters(void)
{
  // Reset timeout flag
  vl53l0x_timeout_flag = 0;
  
  // Start single shot measurement (giống code C thuần)
  vl53_write_reg(VL53L0X_REG_SYSRANGE_START, 0x01);
  
  // Chờ đo xong (timeout 100ms như code C thuần, nhưng có thể điều chỉnh)
  uint8_t timeout = 0;
  uint16_t timeout_limit = (vl53l0x_io_timeout < 100) ? vl53l0x_io_timeout : 100;
  
  while (timeout < timeout_limit) {
    uint8_t status = vl53_read_reg(VL53L0X_REG_SYSRANGE_START);
    if ((status & 0x01) == 0) {
      break; // Đo xong
    }
    _delay_ms(1);
    timeout++;
  }
  
  // Kiểm tra timeout
  if (timeout >= timeout_limit) {
    vl53l0x_timeout_flag = 1;
    return 0; // Timeout
  }
  
  // Đọc kết quả (giống code C thuần)
  uint8_t buffer[12];
  vl53_read_multi(VL53L0X_REG_RESULT_RANGE_STATUS, buffer, 12);
  
  // Distance ở buffer[10] (MSB) và buffer[11] (LSB)
  uint16_t distance = ((uint16_t)buffer[10] << 8) | buffer[11];
  
  // Xử lý kết quả giống code mẫu:
  // - distance >= 8190: OUT OF RANGE
  // - distance == 0: ERROR
  // - Còn lại: OK (20-2000mm)
  
  if (distance >= 8190) {
    return 8190; // Out of range
  }
  
  // distance = 0 có thể là lỗi hoặc hợp lệ (quá gần)
  // Trả về như code C thuần
  return distance;
}

// Kiểm tra timeout (giống sensor.timeoutOccurred())
uint8_t vl53l0x_timeoutOccurred(void)
{
  return vl53l0x_timeout_flag;
}
