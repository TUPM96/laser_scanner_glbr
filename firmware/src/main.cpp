/*
 * 3D Scanner Firmware
 * Controls 2 stepper motors (X and Z) and TF-Luna LiDAR sensor via I2C or UART
 * Sends scan data via Serial communication
 */

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// ==================== CONFIGURATION ====================
// Stepper Motor X (Theta/Rotation) - CNC Shield style
// Note: Pin assignments may differ from original code due to CNC shield layout
#define X_STEP_PIN      2
#define X_DIR_PIN       5
#define X_ENABLE_PIN    8
// Microstepping pins (if available on your CNC shield)
// Original code used: MS1=4, MS2=5, but these conflict with current pin assignments
// Check your CNC shield documentation for MS1/MS2 pin locations

// Stepper Motor Z (Vertical) - CNC Shield style
#define Z_STEP_PIN      4
#define Z_DIR_PIN       7
#define Z_ENABLE_PIN    8
// Microstepping pins (if available on your CNC shield)
// Original code used: MS1=9, MS2=18

// TF-Luna LiDAR Configuration
// Choose communication method: I2C or UART
#define TF_LUNA_USE_UART    true   // Set to true to use UART, false to use I2C

#if TF_LUNA_USE_UART
  // UART Configuration (SoftwareSerial)
  #define TF_RX              12   // Arduino RX pin (connects to TF-Luna TX)
  #define TF_TX              13   // Arduino TX pin (connects to TF-Luna RX, optional)
  SoftwareSerial tfSerial(TF_RX, TF_TX);
#else
  // I2C Configuration
  #define TF_LUNA_I2C_ADDR    0x10  // Default I2C address for TF-Luna
  // I2C pins are fixed on Arduino: SDA=A4, SCL=A5
#endif

// Default Scanning Parameters (can be changed via CONFIG command)
// NOTE: DEFAULT_STEPS_PER_REV is the SINGLE SOURCE OF TRUTH for steps per revolution
// GUI will load this value when connecting. Change this value to change default for both firmware and GUI.
#define DEFAULT_STEPS_PER_REV          1600  // Steps per full rotation (default: 1/8 step mode for NEMA 17)
// NOTE: DEFAULT_THETA_STEPS_PER_REV is the SINGLE SOURCE OF TRUTH for number of measurement points per revolution
// GUI will load this value when connecting. Change this value to change default for both firmware and GUI.
#define DEFAULT_THETA_STEPS_PER_REV    200   // Number of measurement points per revolution (default: 200 points = 1.8° per point)
// NOTE: DEFAULT_Z_STEPS_PER_MM is the SINGLE SOURCE OF TRUTH for Z-axis steps per mm
// GUI will load this value when connecting. Change this value to change default for both firmware and GUI.
// Vitme phi 8: pitch = 8mm, with 1600 steps/rev → 1600 steps / 8mm = 200 steps/mm
#define DEFAULT_Z_STEPS_PER_MM         200  // Steps per mm vertical movement (vitme phi 8: 1600 steps/rev / 8mm = 200 steps/mm)
#define DEFAULT_Z_TRAVEL_MM            200  // Total Z-axis travel in mm (20cm max height)
#define DEFAULT_Z_STEPS_PER_LAYER      400  // Steps between each scan layer (auto-calculated: 2.0mm * 200 steps/mm = 400)
#define DEFAULT_SCAN_DELAY_MS          50   // Delay between measurements

// Limits for theta_steps_per_rev (number of measurement points per revolution)
#define MIN_THETA_STEPS_PER_REV        4    // Minimum points per revolution (at least 4 for basic shape)
#define MAX_THETA_STEPS_PER_REV        3600 // Maximum points per revolution (1 point per 0.1 degree)

// Serial Communication
#define SERIAL_BAUD           115200
#define DATA_DELIMITER        9999  // Delimiter to separate scan layers

// ==================== GLOBAL VARIABLES ====================
bool scanning = false;
bool paused = false;  // Track if scan is paused
bool scan_direction_up = true;  // true = scan upward (bottom to top), false = scan downward (top to bottom)
int current_theta_step = 0;  // Track current rotation position
int paused_layer = 0;  // Save layer position when paused
int paused_step = 0;  // Save step position when paused
int scan_current_layer = 0;  // Current layer during step-by-step scan
int scan_current_step = 0;  // Current step during step-by-step scan
bool step_by_step_mode = false;  // Step-by-step scan mode (controlled by GUI)

// Configurable parameters (can be changed via Serial)
int theta_steps_per_rev = DEFAULT_THETA_STEPS_PER_REV;
int z_steps_per_mm = DEFAULT_Z_STEPS_PER_MM;
int z_travel_mm = DEFAULT_Z_TRAVEL_MM;
int z_steps_per_layer = DEFAULT_Z_STEPS_PER_LAYER;
int scan_delay_ms = DEFAULT_SCAN_DELAY_MS;
float center_distance_cm = 15.0;  // Distance from lidar to center of turntable (used by GUI for coordinate conversion)
int steps_per_rev = DEFAULT_STEPS_PER_REV;  // Steps per full revolution for motor (used for 1 full rotation)

// ==================== FUNCTION DECLARATIONS ====================
void initMotors();
void initTF_Luna();
float readTF_Luna();
void rotateMotorX(int steps);
void rotateMotorZ(int steps);
void moveToTop();
void returnToHome();
void performScan();
void performScanStep();  // Step-by-step scan: rotate 1 step, measure, send data
void resumeScan();
void processConfigCommand(String command);

// ==================== SETUP ====================
void setup() {
  // Initialize Serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize TF-Luna (I2C or UART)
#if TF_LUNA_USE_UART
  tfSerial.begin(115200);
  Serial.println("TF-Luna UART mode initialized");
#else
  Wire.begin();
#endif
  initTF_Luna();
  
  // Initialize stepper motors
  initMotors();
  
  Serial.println("3D Scanner Ready");
  Serial.println("Commands:");
  Serial.println("  START - Begin scanning");
  Serial.println("  STOP - Stop scanning");
  Serial.println("  HOME - Return to home position");
  Serial.println("  MOVE_TO_TOP - Move to top position");
  Serial.println("  CONFIG,theta,z_travel,z_steps/mm,z_steps/layer,delay,center,steps/rev - Set scan parameters");
  Serial.println("  Note: z_steps/layer is auto-calculated from z_layer_height * z_steps/mm in GUI");
  Serial.print("Current config: theta=");
  Serial.print(theta_steps_per_rev);
  Serial.print(" z_travel=");
  Serial.print(z_travel_mm);
  Serial.print("mm z_steps/mm=");
  Serial.print(z_steps_per_mm);
  Serial.print(" z_steps/layer=");
  Serial.print(z_steps_per_layer);
  Serial.print(" delay=");
  Serial.print(scan_delay_ms);
  Serial.println("ms");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "START" || command == "START_UP") {
      // Scan from bottom to top (upward) - Step by step mode
      scanning = true;
      paused = false;
      paused_layer = 0;
      paused_step = 0;
      scan_current_layer = 0;
      scan_current_step = 0;
      scan_direction_up = true;  // Scan upward
      step_by_step_mode = true;  // Enable step-by-step mode
      current_theta_step = 0;  // Reset position
      
      // Don't validate - allow scanning even if config seems invalid
      // User can scan continuously without Z movement if needed
      Serial.println("SCAN_START");
      // Don't call performScan() - wait for SCAN_STEP commands from GUI
    }
    else if (command == "START_DOWN") {
      // Scan from top to bottom (downward) - Step by step mode
      scanning = true;
      paused = false;
      paused_layer = 0;
      paused_step = 0;
      scan_current_layer = 0;
      scan_current_step = 0;
      scan_direction_up = false;  // Scan downward
      step_by_step_mode = true;  // Enable step-by-step mode
      current_theta_step = 0;  // Reset position
      
      // Don't validate - allow scanning even if config seems invalid
      // User can scan continuously without Z movement if needed
      Serial.println("SCAN_START");
      // Don't call performScan() - wait for SCAN_STEP commands from GUI
    }
    else if (command == "SCAN_STEP") {
      // Perform one step of scanning: rotate 1 step, measure, send data
      if (!scanning || !step_by_step_mode) {
        Serial.println("ERROR: Not in step-by-step scan mode!");
        return;
      }
      performScanStep();
    }
    else if (command == "STOP") {
      paused = true;
      scanning = false;
      Serial.println("SCAN_PAUSED");
    }
    else if (command == "RESUME") {
      if (paused) {
        paused = false;
        scanning = true;
        Serial.println("SCAN_RESUMED");
        resumeScan();
        if (!paused) {
          Serial.println("SCAN_COMPLETE");
        }
        scanning = false;
      }
    }
    else if (command == "HOME") {
      returnToHome();
      Serial.println("HOME_COMPLETE");
    }
    else if (command == "MOVE_TO_TOP") {
      Serial.println("Moving to top position...");
      moveToTop();
      Serial.println("MOVE_TO_TOP_COMPLETE");
    }
    else if (command == "TEST") {
      // Test TF-Luna reading
      float distance = readTF_Luna();
      Serial.print("Distance: ");
      Serial.println(distance);
    }
    else if (command == "TEST_POINT") {
      // Measure current point and return angle and distance
      float distance = readTF_Luna();
      // Get current angle from tracked position (based on measurement points, not actual steps)
      float angle = (current_theta_step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
      Serial.print("TEST_POINT:");
      Serial.print(angle, 1);
      Serial.print(",");
      Serial.println(distance, 2);
    }
    else if (command.startsWith("ROTATE,")) {
      // Rotate motor clockwise by specified steps: ROTATE,steps
      String steps_str = command.substring(7);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(X_DIR_PIN, LOW);  // Clockwise direction
        rotateMotorX(steps);
        current_theta_step = (current_theta_step + steps) % theta_steps_per_rev;
        Serial.print("ROTATED:");
        Serial.println(steps);
      } else {
        Serial.print("ROTATE_ERROR: Invalid steps: ");
        Serial.println(steps_str);
      }
    }
    else if (command.startsWith("ROTATE_CCW,")) {
      // Rotate motor X counter-clockwise by specified steps: ROTATE_CCW,steps
      String steps_str = command.substring(11);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(X_DIR_PIN, HIGH);  // Counter-clockwise direction
        rotateMotorX(steps);
        current_theta_step = (current_theta_step - steps + theta_steps_per_rev) % theta_steps_per_rev;
        Serial.print("ROTATED:");
        Serial.print(-steps);  // Negative to indicate CCW
        Serial.println();
      }
    }
    else if (command.startsWith("ROTATE_Z,")) {
      // Rotate motor Z clockwise by specified steps: ROTATE_Z,steps
      String steps_str = command.substring(9);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(Z_DIR_PIN, LOW);  // Clockwise direction (up)
        rotateMotorZ(steps);
        Serial.print("ROTATED_Z:");
        Serial.println(steps);
      } else {
        Serial.print("ROTATE_Z_ERROR: Invalid steps: ");
        Serial.println(steps_str);
      }
    }
    else if (command.startsWith("ROTATE_Z_CCW,")) {
      // Rotate motor Z counter-clockwise by specified steps: ROTATE_Z_CCW,steps
      String steps_str = command.substring(13);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(Z_DIR_PIN, HIGH);  // Counter-clockwise direction (down)
        rotateMotorZ(steps);
        Serial.print("ROTATED_Z:");
        Serial.print(-steps);  // Negative to indicate CCW
        Serial.println();
      }
    }
    else if (command.startsWith("CONFIG,")) {
      // Process configuration command
      processConfigCommand(command);
    }
    else if (command == "GET_CONFIG") {
      // Send current configuration
      Serial.print("CURRENT_CONFIG:");
      Serial.print(theta_steps_per_rev);
      Serial.print(",");
      Serial.print(z_travel_mm);
      Serial.print(",");
      Serial.print(z_steps_per_mm);
      Serial.print(",");
      Serial.print(z_steps_per_layer);
      Serial.print(",");
      Serial.print(scan_delay_ms);
      Serial.print(",");
      Serial.print(center_distance_cm, 1);  // 1 decimal place
      Serial.print(",");
      Serial.println(steps_per_rev);  // Steps per revolution
    }
    else if (command == "READ_LIDAR") {
      // Continuous reading mode - return distance immediately
      float distance = readTF_Luna();
      Serial.print("LIDAR_DISTANCE:");
      Serial.println(distance, 2);
    }
  }
  
  delay(10);
}

// ==================== INITIALIZATION FUNCTIONS ====================
void initMotors() {
  // Configure X motor pins (Theta/Rotation)
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  
  // Configure Z motor pins (Vertical)
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  // Note: Z_ENABLE_PIN might be same as X_ENABLE_PIN on some CNC shields
  
  // Enable motors (LOW = enabled for most stepper drivers)
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  
  // Set initial direction
  digitalWrite(X_DIR_PIN, LOW);  // Clockwise rotation
  digitalWrite(Z_DIR_PIN, LOW);  // Up direction (or adjust based on your setup)
  
  // Initialize step pins to LOW
  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Z_STEP_PIN, LOW);
}

void initTF_Luna() {
#if TF_LUNA_USE_UART
  // TF-Luna initialization via UART
  Serial.println("TF-Luna UART mode");
  delay(100);  // Wait for serial to stabilize
  // Try a test read
  float testDist = readTF_Luna();
  if (testDist > 0 && testDist < 1200) {
    Serial.print("TF-Luna test read: ");
    Serial.print(testDist);
    Serial.println(" cm");
  } else {
    Serial.println("TF-Luna UART initialized (waiting for data)");
  }
#else
  // TF-Luna initialization via I2C
  // The sensor should auto-initialize on power-up
  delay(100);
  
  // Try to read from sensor to verify connection
  Wire.beginTransmission(TF_LUNA_I2C_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("TF-Luna detected on I2C");
    delay(50);
    // Try a test read
    float testDist = readTF_Luna();
    if (testDist > 0) {
      Serial.print("TF-Luna test read: ");
      Serial.print(testDist);
      Serial.println(" cm");
    }
  } else {
    Serial.print("Warning: TF-Luna not detected (Error: ");
    Serial.print(error);
    Serial.println(")");
  }
#endif
}

// ==================== TF-LUNA FUNCTIONS ====================
float readTF_Luna() {
#if TF_LUNA_USE_UART
  // TF-Luna UART protocol:
  // Frame format: 0x59 0x59 [DistL] [DistH] [FluxL] [FluxH] [TempL] [TempH] [Checksum]
  // Need at least 9 bytes for a complete frame
  
  // Wait for data with timeout (max 100ms)
  unsigned long startTime = millis();
  while (tfSerial.available() < 9 && (millis() - startTime) < 100) {
    delay(1);
  }
  
  if (tfSerial.available() < 9) {
    // Not enough data, clear buffer and return 0
    while (tfSerial.available() > 0) {
      tfSerial.read();
    }
    return 0.0; // Return 0 for invalid reading (will be skipped in processing)
  }
  
  // Synchronize to frame header (0x59 0x59)
  // Search for header in available data - clear old data until we find valid header
  uint8_t byte1, byte2;
  bool headerFound = false;
  int attempts = 0;
  const int maxAttempts = 50;  // Max attempts to find header (clear up to 50 bytes)
  
  // First, try to find header in current buffer
  while (attempts < maxAttempts && tfSerial.available() >= 9) {
    byte1 = tfSerial.read();
    
    if (byte1 == 0x59) {
      // First byte matches, check second byte
      if (tfSerial.available() >= 8) {
        byte2 = tfSerial.read();
        if (byte2 == 0x59) {
          // Found header, read the rest of the frame
          headerFound = true;
          break;
        } else {
          // Second byte doesn't match, continue searching from current position
          // Don't discard byte2, it might be the start of a new header
          if (byte2 == 0x59) {
            // Actually, byte2 is also 0x59, so this could be a header
            // But we already read it, so we need to check if next byte is also 0x59
            // For now, continue searching
          }
        }
      } else {
        // Not enough data, wait a bit and retry
        delay(5);
        if (tfSerial.available() >= 8) {
          byte2 = tfSerial.read();
          if (byte2 == 0x59) {
            headerFound = true;
            break;
          }
        }
      }
    }
    attempts++;
  }
  
  if (!headerFound) {
    // Header not found, clear buffer and return 0
    while (tfSerial.available() > 0) {
      tfSerial.read();
    }
    return 0.0; // Return 0 for invalid reading (will be skipped in processing)
  }
  
  // Read the rest of the frame (7 bytes remaining)
  if (tfSerial.available() < 7) {
    // Not enough data, clear and return 0
    while (tfSerial.available() > 0) {
      tfSerial.read();
    }
    return 0.0; // Return 0 for invalid reading (will be skipped in processing)
  }
  
  // Read distance (2 bytes)
  uint8_t distL = tfSerial.read();
  uint8_t distH = tfSerial.read();
  
  // Read signal strength/flux (2 bytes)
  uint8_t fluxL = tfSerial.read();
  uint8_t fluxH = tfSerial.read();
  
  // Read temperature (2 bytes)
  uint8_t tempL = tfSerial.read();
  uint8_t tempH = tfSerial.read();
  
  // Read checksum (not used for validation in this implementation)
  tfSerial.read();
  
  // Combine distance bytes
  uint16_t distance_cm = (distH << 8) | distL;
  
  // Combine signal strength bytes
  uint16_t signalStrength = (fluxH << 8) | fluxL;
  
  // Check signal quality - minimum threshold
  if (signalStrength < 50 || distance_cm == 0 || distance_cm > 1200) {
    return 0.0; // Return 0 for invalid reading (will be skipped in processing)
  }
  
  // TF-Luna returns distance in cm via UART
  return (float)distance_cm; // Return in cm
#else
  // TF-Luna I2C protocol:
  // The sensor continuously outputs data, we read from the data registers
  // Register 0x00: Distance low byte
  // Register 0x01: Distance high byte  
  // Register 0x02: Flux/Strength low byte
  // Register 0x03: Flux/Strength high byte
  // Register 0x04: Temperature low byte
  // Register 0x05: Temperature high byte
  
  uint16_t distance = 0;
  uint16_t signalStrength = 0;
  
  // Read distance (registers 0x00-0x01)
  Wire.beginTransmission(TF_LUNA_I2C_ADDR);
  Wire.write(0x00);  // Start from register 0
  if (Wire.endTransmission() != 0) {
    return 0.0; // Return 0 for communication error (will be skipped in processing)
  }
  
  // Request 6 bytes (distance, flux, temperature)
  uint8_t bytesReceived = Wire.requestFrom(TF_LUNA_I2C_ADDR, 6);
  if (bytesReceived < 6) {
    return 0.0; // Return 0 if not enough data (will be skipped in processing)
  }
  
  // Read distance (2 bytes)
  uint8_t distLow = Wire.read();
  uint8_t distHigh = Wire.read();
  distance = (distHigh << 8) | distLow;
  
  // Read signal strength/flux (2 bytes)
  uint8_t fluxLow = Wire.read();
  uint8_t fluxHigh = Wire.read();
  signalStrength = (fluxHigh << 8) | fluxLow;
  
  // Read temperature (2 bytes) - not used but must read to clear buffer
  Wire.read();
  Wire.read();
  
  // Check signal quality - minimum threshold (relaxed for testing)
  if (signalStrength < 50 || distance == 0 || distance > 1200) {
    return 0.0; // Return 0 for invalid reading (will be skipped in processing)
  }
  
  // TF-Luna returns distance in cm
  return (float)distance; // Return in cm
#endif
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
void rotateMotorX(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(500);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(500);
  }
}

void rotateMotorZ(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(Z_STEP_PIN, LOW);
    delayMicroseconds(500);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(500);
  }
}

void moveToTop() {
  // Move Z-axis to top position (for scanning from top to bottom)
  digitalWrite(Z_DIR_PIN, LOW); // Up direction
  int totalZSteps = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  for (int i = 0; i < totalZSteps; i++) {
    rotateMotorZ(z_steps_per_layer);
    delay(10);
  }
}

// ==================== STEP-BY-STEP SCANNING FUNCTION ====================
void performScanStep() {
  // Calculate number of layers (but don't enforce - allow continuous scanning)
  int zLayers = 0;
  if (z_steps_per_layer > 0) {
    zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  }
  // If zLayers is 0 or invalid, set to a large number to allow continuous scanning
  if (zLayers <= 0) {
    zLayers = 1000;  // Allow scanning to continue
  }
  
  // Check if we've completed one full rotation (one layer)
  if (scan_current_step >= theta_steps_per_rev) {
    // Move to next layer
    scan_current_step = 0;
    scan_current_layer++;
    
    // Check if all layers are done (only if we've scanned at least one full layer)
    if (scan_current_layer >= zLayers && scan_current_layer > 0) {
      Serial.println("SCAN_COMPLETE");
      scanning = false;
      step_by_step_mode = false;
      return;
    }
    
    // Move Z motor to next layer (only if z_steps_per_layer > 0)
    if (z_steps_per_layer > 0) {
      if (scan_direction_up) {
        digitalWrite(Z_DIR_PIN, LOW);  // Up direction
      } else {
        digitalWrite(Z_DIR_PIN, HIGH);  // Down direction
      }
      rotateMotorZ(z_steps_per_layer);
      digitalWrite(Z_DIR_PIN, LOW);  // Reset direction
    }
    
    // Send layer delimiter
    Serial.println(DATA_DELIMITER);
    return;  // Wait for next SCAN_STEP command
  }
  
  // Rotate X motor by 1 step only
  digitalWrite(X_DIR_PIN, LOW);  // Clockwise direction
  rotateMotorX(1);  // Rotate exactly 1 step
  
  // Update current step
  scan_current_step++;
  current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;
  
  // Small delay for LiDAR stabilization
  delay(20);
  
  // Read distance from TF-Luna
  float distance = readTF_Luna();
  
  // Calculate angle
  float angle = (scan_current_step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
  
  // Send data: "LAYER,STEP,DISTANCE,ANGLE"
  Serial.print(scan_current_layer);
  Serial.print(",");
  Serial.print(scan_current_step - 1);  // Step index (0-based)
  Serial.print(",");
  Serial.print(distance, 2);
  Serial.print(",");
  Serial.println(angle, 1);
}

void returnToHome() {
  // Return Z-axis to home position (bottom)
  digitalWrite(Z_DIR_PIN, HIGH); // Reverse direction (down)
  int totalZSteps = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  for (int i = 0; i < totalZSteps; i++) {
    rotateMotorZ(z_steps_per_layer);
    delay(10);
  }
  digitalWrite(Z_DIR_PIN, LOW); // Reset direction to up
}

// ==================== SCANNING FUNCTION ====================
void performScan() {
  // Send error message if function is called but scanning is false
  if (!scanning) {
    Serial.println("ERROR: performScan() called but scanning=false!");
    return;
  }
  
  // Calculate number of layers
  int zLayers = 0;
  if (z_steps_per_layer > 0) {
    zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  }
  
  // Send error message with details if zLayers is invalid
  if (zLayers <= 0) {
    Serial.print("ERROR: zLayers=");
    Serial.print(zLayers);
    Serial.print(" (z_travel_mm=");
    Serial.print(z_travel_mm);
    Serial.print(", z_steps_per_mm=");
    Serial.print(z_steps_per_mm);
    Serial.print(", z_steps_per_layer=");
    Serial.print(z_steps_per_layer);
    Serial.println("), cannot scan!");
    scanning = false;
    return;
  }
  
  current_theta_step = 0;  // Reset position at start of scan
  
  // Note: User should move to top manually before starting scan
  // Scan will proceed from current position downward
  
  // Calculate steps per measurement point to complete 1 full revolution
  // steps_per_rev: actual steps needed for 1 full rotation
  // theta_steps_per_rev: number of measurement points in 1 full rotation
  // Use integer division and accumulate remainder to ensure exact 1 revolution
  int steps_per_point = steps_per_rev / theta_steps_per_rev;
  int remainder = steps_per_rev % theta_steps_per_rev;  // Remaining steps to distribute
  
  // Send error message with details if steps_per_point is invalid
  if (steps_per_point < 1) {
    Serial.print("ERROR: steps_per_point=");
    Serial.print(steps_per_point);
    Serial.print(" (steps_per_rev=");
    Serial.print(steps_per_rev);
    Serial.print(", theta_steps_per_rev=");
    Serial.print(theta_steps_per_rev);
    Serial.println("), cannot scan!");
    scanning = false;
    return;
  }
  
  // Send confirmation that scan is starting
  Serial.print("SCAN_INFO: Starting scan - zLayers=");
  Serial.print(zLayers);
  Serial.print(", steps_per_point=");
  Serial.print(steps_per_point);
  Serial.print(", remainder=");
  Serial.println(remainder);
  
  // Scan each layer
  Serial.println("SCAN_INFO: Entering scan loop");
  for (int layer = 0; layer < zLayers && scanning && !paused; layer++) {
    Serial.print("SCAN_INFO: Layer ");
    Serial.print(layer);
    Serial.print("/");
    Serial.print(zLayers);
    Serial.print(", scanning=");
    Serial.print(scanning);
    Serial.print(", paused=");
    Serial.println(paused);
    
    // Rotate and scan one full rotation
    for (int step = 0; step < theta_steps_per_rev && scanning && !paused; step++) {
      // Check scanning status before each step
      if (!scanning || paused) {
        Serial.print("SCAN_INFO: Stopped at layer=");
        Serial.print(layer);
        Serial.print(", step=");
        Serial.println(step);
        break;
      }
      
      // Calculate steps for this point (distribute remainder evenly)
      // Add 1 extra step to the first 'remainder' points to ensure exact 1 revolution
      int steps_this_point = steps_per_point;
      if (step < remainder) {
        steps_this_point++;
      }
      
      // Ensure motor always rotates at least 1 step
      // This prevents motor from not moving if calculation results in 0
      if (steps_this_point <= 0) {
        steps_this_point = 1;  // Force at least 1 step
      }
      
      // Rotate X motor by calculated steps to reach next measurement position
      // Always rotate at least 1 step per measurement point
      if (step == 0 && layer == 0) {
        Serial.print("SCAN_INFO: First rotation - steps_this_point=");
        Serial.println(steps_this_point);
      }
      digitalWrite(X_DIR_PIN, LOW);  // Ensure direction is set (clockwise)
      rotateMotorX(steps_this_point);
      if (step == 0 && layer == 0) {
        Serial.println("SCAN_INFO: Motor X rotated");
      }
      if (!scanning || paused) break;  // Check after motor movement
      
      current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;
      
      // Check scanning during delay (split delay into smaller chunks for responsive stop)
      int delay_remaining = scan_delay_ms;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);  // Check every 10ms
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        // Save position when paused
        paused_layer = layer;
        paused_step = step;
        break;  // Exit if stopped during delay
      }
      
      // Small additional delay to ensure LiDAR has fresh data after motor movement
      delay(20);  // 20ms for LiDAR to stabilize and get synchronized reading
      
      // Read distance from TF-Luna (synchronized frame reading)
      float distance = readTF_Luna();
      
      // Only send data if not paused
      if (!paused) {
        // Send data via Serial: format: "LAYER,STEP,DISTANCE,ANGLE"
        // ANGLE is current rotation angle in degrees
        float angle = (step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
        Serial.print(layer);
        Serial.print(",");
        Serial.print(step);
        Serial.print(",");
        Serial.print(distance, 2);  // 2 decimal places
        Serial.print(",");
        Serial.println(angle, 1);  // 1 decimal place for angle
      }
    }
    
    // Save position if paused
    if (paused) {
      paused_layer = layer;
      paused_step = 0;
      break;
    }
    
    // Move Z motor down one layer (after X motor completes 1 full revolution)
    // Scanning from top to bottom
    if (layer < zLayers - 1 && scanning && !paused) {
      if (!scanning || paused) break;  // Check before moving Z
      
      digitalWrite(Z_DIR_PIN, HIGH);  // Set direction to down
      rotateMotorZ(z_steps_per_layer);  // Move down by configured layer height
      digitalWrite(Z_DIR_PIN, LOW);  // Reset direction to up (for next scan)
      if (!scanning || paused) break;  // Check after Z movement
      
      // Check scanning during delay
      int delay_remaining = 100;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);  // Check every 10ms
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = 0;
        break;  // Exit if stopped during delay
      }
      
      // Send delimiter to indicate new layer
      if (!paused) {
        Serial.println(DATA_DELIMITER);
      }
    }
  }
  
  // Return to home only if scan completed (not paused)
  if (scanning && !paused) {
    returnToHome();
  }
}

// ==================== RESUME SCANNING FUNCTION ====================
void resumeScan() {
  int zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  
  // Resume from saved position
  int start_layer = paused_layer;
  int start_step = paused_step;
  
  // If resuming from layer 0, move to top first (scan from top to bottom)
  if (start_layer == 0 && start_step == 0) {
    Serial.println("Resuming: Moving to top position...");
    moveToTop();
    delay(500);  // Wait for motor to settle
  }
  
  // Calculate steps per measurement point to complete 1 full revolution
  // steps_per_rev: actual steps needed for 1 full rotation
  // theta_steps_per_rev: number of measurement points in 1 full rotation
  // Use integer division and distribute remainder to ensure exact 1 revolution
  int steps_per_point = steps_per_rev / theta_steps_per_rev;
  int remainder = steps_per_rev % theta_steps_per_rev;  // Remaining steps to distribute
  
  // Scan each layer starting from saved position
  for (int layer = start_layer; layer < zLayers && scanning && !paused; layer++) {
    // Determine starting step for this layer
    int step_start = (layer == start_layer) ? start_step : 0;
    
    // Rotate and scan one full rotation
    for (int step = step_start; step < theta_steps_per_rev && scanning && !paused; step++) {
      if (!scanning || paused) break;  // Check scanning status before each step
      
      // Calculate steps for this point (distribute remainder evenly)
      // Add 1 extra step to the first 'remainder' points to ensure exact 1 revolution
      int steps_this_point = steps_per_point;
      if (step < remainder) {
        steps_this_point++;
      }
      
      // Rotate X motor by calculated steps to reach next measurement position
      rotateMotorX(steps_this_point);
      if (!scanning || paused) break;  // Check after motor movement
      
      current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;
      
      // Check scanning during delay (split delay into smaller chunks for responsive stop)
      int delay_remaining = scan_delay_ms;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);  // Check every 10ms
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        // Save position when paused
        paused_layer = layer;
        paused_step = step;
        break;  // Exit if stopped during delay
      }
      
      // Small additional delay to ensure LiDAR has fresh data after motor movement
      delay(20);  // 20ms for LiDAR to stabilize and get synchronized reading
      
      // Read distance from TF-Luna (synchronized frame reading)
      float distance = readTF_Luna();
      
      // Only send data if not paused
      if (!paused) {
        // Send data via Serial: format: "LAYER,STEP,DISTANCE,ANGLE"
        // ANGLE is current rotation angle in degrees
        float angle = (step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
        Serial.print(layer);
        Serial.print(",");
        Serial.print(step);
        Serial.print(",");
        Serial.print(distance, 2);  // 2 decimal places
        Serial.print(",");
        Serial.println(angle, 1);  // 1 decimal place for angle
      }
    }
    
    // Save position if paused
    if (paused) {
      paused_layer = layer;
      paused_step = 0;
      break;
    }
    
    // Move Z motor down one layer (after X motor completes 1 full revolution)
    // Scanning from top to bottom
    if (layer < zLayers - 1 && scanning && !paused) {
      if (!scanning || paused) break;  // Check before moving Z
      
      digitalWrite(Z_DIR_PIN, HIGH);  // Set direction to down
      rotateMotorZ(z_steps_per_layer);  // Move down by configured layer height
      digitalWrite(Z_DIR_PIN, LOW);  // Reset direction to up (for next scan)
      if (!scanning || paused) break;  // Check after Z movement
      
      // Check scanning during delay
      int delay_remaining = 100;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);  // Check every 10ms
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = 0;
        break;  // Exit if stopped during delay
      }
      
      // Send delimiter to indicate new layer
      if (!paused) {
        Serial.println(DATA_DELIMITER);
      }
    }
  }
  
  // Return to home only if scan completed (not paused)
  if (scanning && !paused) {
    returnToHome();
  }
}

// ==================== CONFIGURATION FUNCTION ====================
void processConfigCommand(String command) {
  // Parse command: CONFIG,theta_steps,z_travel,z_steps_per_mm,z_steps_per_layer,scan_delay,center_distance,steps_per_rev
  // Remove "CONFIG," prefix
  String params = command.substring(7);
  
  // Parse comma-separated values (7 parameters: 6 int + 1 float)
  int commaIndex = 0;
  int intValues[6];
  float centerDist = 10.3;  // Default value
  int stepsPerRev = DEFAULT_STEPS_PER_REV;  // Default value
  
  // Parse first 5 integer values (theta_steps, z_travel, z_steps_per_mm, z_steps_per_layer, scan_delay)
  for (int i = 0; i < 5; i++) {
    commaIndex = params.indexOf(',');
    if (commaIndex == -1) {
      // Last integer parameter
      intValues[i] = params.toInt();
      params = "";  // No more params
      break;
    } else {
      intValues[i] = params.substring(0, commaIndex).toInt();
      params = params.substring(commaIndex + 1);
    }
  }
  
  // Parse center_distance (float) if present
  if (params.length() > 0) {
    commaIndex = params.indexOf(',');
    if (commaIndex == -1) {
      // Only center_distance, no steps_per_rev
      centerDist = params.toFloat();
      params = "";
    } else {
      centerDist = params.substring(0, commaIndex).toFloat();
      params = params.substring(commaIndex + 1);
      // Parse steps_per_rev if present
      if (params.length() > 0) {
        stepsPerRev = params.toInt();
      }
    }
  }
  
  // Validate and set values
  // Check theta_steps_per_rev limits
  if (intValues[0] < MIN_THETA_STEPS_PER_REV || intValues[0] > MAX_THETA_STEPS_PER_REV) {
    Serial.print("CONFIG_ERROR: theta_steps must be between ");
    Serial.print(MIN_THETA_STEPS_PER_REV);
    Serial.print(" and ");
    Serial.print(MAX_THETA_STEPS_PER_REV);
    Serial.println();
    return;
  }
  
  // Check that theta_steps_per_rev doesn't exceed steps_per_rev (to ensure steps_per_point >= 1)
  if (intValues[0] > stepsPerRev) {
    Serial.print("CONFIG_ERROR: theta_steps (");
    Serial.print(intValues[0]);
    Serial.print(") cannot exceed steps_per_rev (");
    Serial.print(stepsPerRev);
    Serial.println(")");
    return;
  }
  
  if (intValues[0] > 0 && intValues[1] > 0 && intValues[2] > 0 && intValues[3] > 0 && intValues[4] >= 0 && centerDist > 0 && stepsPerRev > 0) {
    theta_steps_per_rev = intValues[0];
    z_travel_mm = intValues[1];
    z_steps_per_mm = intValues[2];
    z_steps_per_layer = intValues[3];
    scan_delay_ms = intValues[4];
    center_distance_cm = centerDist;
    steps_per_rev = stepsPerRev;
    
    Serial.print("CONFIG_OK:");
    Serial.print(" theta=");
    Serial.print(theta_steps_per_rev);
    Serial.print(" z_travel=");
    Serial.print(z_travel_mm);
    Serial.print("mm z_steps/mm=");
    Serial.print(z_steps_per_mm);
    Serial.print(" z_steps/layer=");
    Serial.print(z_steps_per_layer);
    Serial.print(" delay=");
    Serial.print(scan_delay_ms);
    Serial.print("ms center=");
    Serial.print(center_distance_cm, 1);
    Serial.print("cm steps/rev=");
    Serial.println(steps_per_rev);
  } else {
    Serial.println("CONFIG_ERROR: Invalid values");
  }
}

