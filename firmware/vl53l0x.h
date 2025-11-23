/*
  vl53l0x.h - Header file cho driver sensor VL53L0X
  Dựa trên code C thuần
  
  Part of Grbl
  Copyright (c) 2024
*/

#ifndef vl53l0x_h
#define vl53l0x_h

#include "grbl.h"
#include "i2c.h"

// Địa chỉ I2C của VL53L0X (7-bit: 0x29)
#define VL53L0X_I2C_ADDR 0x29

// Địa chỉ các register của VL53L0X
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID    0xC0
#define VL53L0X_REG_SYSRANGE_START             0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS        0x14
#define VL53L0X_REG_RESULT_RANGE_VAL           0x1E

// API giống thư viện Pololu
// sensor.init() -> trả về 1 nếu thành công, 0 nếu lỗi
uint8_t vl53l0x_init(void);

// sensor.setTimeout(500) - không cần trong single shot mode
void vl53l0x_setTimeout(uint16_t timeout);

// sensor.startContinuous(100) - không cần trong single shot mode
void vl53l0x_startContinuous(uint16_t period_ms);

// sensor.readRangeContinuousMillimeters() -> trả về distance (mm)
// Trong single shot mode, mỗi lần gọi sẽ trigger một lần đo mới
uint16_t vl53l0x_readRangeContinuousMillimeters(void);

// sensor.timeoutOccurred() -> trả về 1 nếu timeout, 0 nếu không
uint8_t vl53l0x_timeoutOccurred(void);

#endif
