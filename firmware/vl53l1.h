/*
  vl53l1.h - Header file cho driver sensor VL53L1
  Dựa trên cấu trúc VL53L0X với register addresses của VL53L1
  
  Part of Grbl
  Copyright (c) 2024
*/

#ifndef vl53l1_h
#define vl53l1_h

#include "grbl.h"
#include "i2c.h"

// Địa chỉ I2C của VL53L1 (7-bit: 0x29, giống VL53L0X)
#define VL53L1_I2C_ADDR 0x29

// Địa chỉ các register của VL53L1
#define VL53L1_REG_RESULT_RANGE_STATUS        0x0089
#define VL53L1_REG_RESULT_DISTANCE            0x0096  // Register chứa distance (16-bit)

// API giống VL53L0X để dễ sử dụng
// sensor.init() -> trả về 1 nếu thành công, 0 nếu lỗi
uint8_t vl53l1_init(void);

// sensor.setTimeout(500)
void vl53l1_setTimeout(uint16_t timeout);

// sensor.startContinuous(100)
void vl53l1_startContinuous(uint16_t period_ms);

// sensor.readRangeContinuousMillimeters() -> trả về distance (mm)
uint16_t vl53l1_readRangeContinuousMillimeters(void);

// sensor.timeoutOccurred() -> trả về 1 nếu timeout, 0 nếu không
uint8_t vl53l1_timeoutOccurred(void);

#endif

