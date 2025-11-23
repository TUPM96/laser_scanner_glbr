/*
  defaults.h - Header file chọn cấu hình mặc định
  
  CHỨC NĂNG:
  - File chọn cấu hình mặc định cho các loại máy khác nhau
  - Từ DIY CNC mills đến máy CNC chuyển đổi
  - Các cấu hình được cung cấp bởi người dùng
  - Đảm bảo chỉ có một DEFAULTS_XXX được định nghĩa trong config.h
  
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
*/

#ifndef defaults_h

// Chỉ định nghĩa DEFAULT_XXX với nơi tìm file default_XXX.h tương ứng
// Không #define defaults_h ở đây, để file được chọn làm việc đó
// Ngăn include nhiều hơn một file

#ifdef DEFAULTS_GENERIC
  // Grbl generic default settings. Should work across different machines.
  #include "defaults/defaults_generic.h"
#endif

#ifdef DEFAULTS_SHERLINE_5400
  // Description: Sherline 5400 mill with three NEMA 23 Keling  KL23H256-21-8B 185 oz-in stepper motors,
  // driven by three Pololu A4988 stepper drivers with a 30V, 6A power supply at 1.5A per winding.
  #include "defaults/defaults_sherline.h"
#endif

#ifdef DEFAULTS_SHAPEOKO
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield with a 24V, 4.2A power supply.
  #include "defaults/defaults_shapeoko.h"
#endif

#ifdef DEFAULTS_SHAPEOKO_2
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield at 28V.
  #include "defaults/defaults_shapeoko2.h"
#endif

#ifdef DEFAULTS_SHAPEOKO_3
  // Description: Shapeoko CNC mill with three NEMA 23 stepper motors, driven by CarbideMotion
  #include "defaults/defaults_shapeoko3.h"
#endif

#ifdef DEFAULTS_X_CARVE_500MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #include "defaults/defaults_x_carve_500mm.h"
#endif

#ifdef DEFAULTS_X_CARVE_1000MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #include "defaults/defaults_x_carve_1000mm.h"
#endif

#ifdef DEFAULTS_ZEN_TOOLWORKS_7x7
  // Description: Zen Toolworks 7x7 mill with three Shinano SST43D2121 65oz-in NEMA 17 stepper motors.
  // Leadscrew is different from some ZTW kits, where most are 1.25mm/rev rather than 8.0mm/rev here.
  // Driven by 30V, 6A power supply and TI DRV8811 stepper motor drivers.
  #include "defaults/defaults_zen_toolworks_7x7.h"
#endif

#ifdef DEFAULTS_OXCNC
  // Grbl settings for OpenBuilds OX CNC Machine
  // http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  #include "defaults/defaults_oxcnc.h"
#endif

#ifdef DEFAULTS_SIMULATOR
  // Settings only for Grbl Simulator (www.github.com/grbl/grbl-sim)
  #include "defaults/defaults_simulator.h"
#endif

#endif
