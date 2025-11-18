#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ==================== CONFIGURATION ====================
#define X_STEP_PIN      2
#define X_DIR_PIN       5
#define X_ENABLE_PIN    8
#define Z_STEP_PIN      4
#define Z_DIR_PIN       7
#define Z_ENABLE_PIN    8

#define DEFAULT_STEPS_PER_REV          1600
#define DEFAULT_THETA_STEPS_PER_REV    200
#define DEFAULT_Z_STEPS_PER_MM         200
#define DEFAULT_Z_TRAVEL_MM            200
#define DEFAULT_Z_STEPS_PER_LAYER      400
#define DEFAULT_SCAN_DELAY_MS          50
#define MIN_THETA_STEPS_PER_REV        4
#define MAX_THETA_STEPS_PER_REV        3600
#define SERIAL_BAUD                    115200
#define DATA_DELIMITER                 9999

bool scanning = false;
bool paused = false;
bool scan_direction_up = true;
int current_theta_step = 0;
int paused_layer = 0;
int paused_step = 0;
int scan_current_layer = 0;
int scan_current_step = 0;
bool step_by_step_mode = false;

int theta_steps_per_rev = DEFAULT_THETA_STEPS_PER_REV;
int z_steps_per_mm = DEFAULT_Z_STEPS_PER_MM;
int z_travel_mm = DEFAULT_Z_TRAVEL_MM;
int z_steps_per_layer = DEFAULT_Z_STEPS_PER_LAYER;
int scan_delay_ms = DEFAULT_SCAN_DELAY_MS;
float center_distance_cm = 15.0;
int steps_per_rev = DEFAULT_STEPS_PER_REV;

VL53L0X sensor;

// ==================== FUNCTION DECLARATIONS ====================
void initMotors();
void initVL53L0X();
float readVL53L0X();
void rotateMotorX(int steps);
void rotateMotorZ(int steps);
void moveToTop();
void returnToHome();
void performScan();
void performScanStep();
void resumeScan();
void processConfigCommand(String command);

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  Wire.begin();
  initVL53L0X();
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

// ==================== VL53L0X FUNCTIONS ====================
void initVL53L0X() {
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1) {}
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  Serial.println("VL53L0X Initialized");
}

float readVL53L0X() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    return 0.0;
  }
  if (distance > 1200 || distance == 0) {
    return 0.0;
  }
  return distance / 10.0; // trả về cm
}

// ==================== MAIN LOOP ====================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "START" || command == "START_UP") {
      scanning = true;
      paused = false;
      paused_layer = 0;
      paused_step = 0;
      scan_current_layer = 0;
      scan_current_step = 0;
      scan_direction_up = true;
      step_by_step_mode = true;
      current_theta_step = 0;
      Serial.println("SCAN_START");
    }
    else if (command == "START_DOWN") {
      scanning = true;
      paused = false;
      paused_layer = 0;
      paused_step = 0;
      scan_current_layer = 0;
      scan_current_step = 0;
      scan_direction_up = false;
      step_by_step_mode = true;
      current_theta_step = 0;
      Serial.println("SCAN_START");
    }
    else if (command == "SCAN_STEP") {
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
      float distance = readVL53L0X();
      Serial.print("Distance: ");
      Serial.println(distance);
    }
    else if (command == "TEST_POINT") {
      float distance = readVL53L0X();
      float angle = (current_theta_step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
      Serial.print("TEST_POINT:");
      Serial.print(angle, 1);
      Serial.print(",");
      Serial.println(distance, 2);
    }
    else if (command.startsWith("ROTATE,")) {
      String steps_str = command.substring(7);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(X_DIR_PIN, LOW);
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
      String steps_str = command.substring(11);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(X_DIR_PIN, HIGH);
        rotateMotorX(steps);
        current_theta_step = (current_theta_step - steps + theta_steps_per_rev) % theta_steps_per_rev;
        Serial.print("ROTATED:");
        Serial.print(-steps);
        Serial.println();
      }
    }
    else if (command.startsWith("ROTATE_Z,")) {
      String steps_str = command.substring(9);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(Z_DIR_PIN, LOW);
        rotateMotorZ(steps);
        Serial.print("ROTATED_Z:");
        Serial.println(steps);
      } else {
        Serial.print("ROTATE_Z_ERROR: Invalid steps: ");
        Serial.println(steps_str);
      }
    }
    else if (command.startsWith("ROTATE_Z_CCW,")) {
      String steps_str = command.substring(13);
      int steps = steps_str.toInt();
      if (steps > 0) {
        digitalWrite(Z_DIR_PIN, HIGH);
        rotateMotorZ(steps);
        Serial.print("ROTATED_Z:");
        Serial.print(-steps);
        Serial.println();
      }
    }
    else if (command.startsWith("CONFIG,")) {
      processConfigCommand(command);
    }
    else if (command == "GET_CONFIG") {
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
      Serial.print(center_distance_cm, 1);
      Serial.print(",");
      Serial.println(steps_per_rev);
    }
    else if (command == "READ_LIDAR") {
      float distance = readVL53L0X();
      Serial.print("LIDAR_DISTANCE:");
      Serial.println(distance, 2);
    }
  }
  delay(10);
}

// ==================== INITIALIZATION FUNCTIONS ====================
void initMotors() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);

  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Z_DIR_PIN, LOW);

  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Z_STEP_PIN, LOW);
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
  digitalWrite(Z_DIR_PIN, LOW);
  int totalZSteps = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  for (int i = 0; i < totalZSteps; i++) {
    rotateMotorZ(z_steps_per_layer);
    delay(10);
  }
}

// ==================== STEP-BY-STEP SCANNING FUNCTION ====================
void performScanStep() {
  int zLayers = 0;
  if (z_steps_per_layer > 0) {
    zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  }
  if (zLayers <= 0) {
    zLayers = 1000;
  }

  if (scan_current_step >= theta_steps_per_rev) {
    scan_current_step = 0;
    scan_current_layer++;

    if (scan_current_layer >= zLayers && scan_current_layer > 0) {
      Serial.println("SCAN_COMPLETE");
      scanning = false;
      step_by_step_mode = false;
      return;
    }

    if (z_steps_per_layer > 0) {
      if (scan_direction_up) {
        digitalWrite(Z_DIR_PIN, LOW);
      } else {
        digitalWrite(Z_DIR_PIN, HIGH);
      }
      rotateMotorZ(z_steps_per_layer);
      digitalWrite(Z_DIR_PIN, LOW);
    }

    Serial.println(DATA_DELIMITER);
    return;
  }

  digitalWrite(X_DIR_PIN, LOW);
  rotateMotorX(1);

  scan_current_step++;
  current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;

  delay(20);

  float distance = readVL53L0X();

  float angle = (scan_current_step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;

  Serial.print(scan_current_layer);
  Serial.print(",");
  Serial.print(scan_current_step - 1);
  Serial.print(",");
  Serial.print(distance, 2);
  Serial.print(",");
  Serial.println(angle, 1);
}

void returnToHome() {
  digitalWrite(Z_DIR_PIN, HIGH);
  int totalZSteps = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  for (int i = 0; i < totalZSteps; i++) {
    rotateMotorZ(z_steps_per_layer);
    delay(10);
  }
  digitalWrite(Z_DIR_PIN, LOW);
}

// ==================== SCANNING FUNCTION ====================
void performScan() {
  if (!scanning) {
    Serial.println("ERROR: performScan() called but scanning=false!");
    return;
  }

  int zLayers = 0;
  if (z_steps_per_layer > 0) {
    zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  }

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

  current_theta_step = 0;

  int steps_per_point = steps_per_rev / theta_steps_per_rev;
  int remainder = steps_per_rev % theta_steps_per_rev;

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

  Serial.print("SCAN_INFO: Starting scan - zLayers=");
  Serial.print(zLayers);
  Serial.print(", steps_per_point=");
  Serial.print(steps_per_point);
  Serial.print(", remainder=");
  Serial.println(remainder);

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

    for (int step = 0; step < theta_steps_per_rev && scanning && !paused; step++) {
      if (!scanning || paused) {
        Serial.print("SCAN_INFO: Stopped at layer=");
        Serial.print(layer);
        Serial.print(", step=");
        Serial.println(step);
        break;
      }

      int steps_this_point = steps_per_point;
      if (step < remainder) {
        steps_this_point++;
      }

      if (steps_this_point <= 0) {
        steps_this_point = 1;
      }

      digitalWrite(X_DIR_PIN, LOW);
      rotateMotorX(steps_this_point);

      if (!scanning || paused) break;
      current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;

      int delay_remaining = scan_delay_ms;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = step;
        break;
      }

      delay(20);

      float distance = readVL53L0X();

      if (!paused) {
        float angle = (step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
        Serial.print(layer);
        Serial.print(",");
        Serial.print(step);
        Serial.print(",");
        Serial.print(distance, 2);
        Serial.print(",");
        Serial.println(angle, 1);
      }
    }

    if (paused) {
      paused_layer = layer;
      paused_step = 0;
      break;
    }

    if (layer < zLayers - 1 && scanning && !paused) {
      if (!scanning || paused) break;

      digitalWrite(Z_DIR_PIN, HIGH);
      rotateMotorZ(z_steps_per_layer);
      digitalWrite(Z_DIR_PIN, LOW);
      if (!scanning || paused) break;

      int delay_remaining = 100;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = 0;
        break;
      }

      if (!paused) {
        Serial.println(DATA_DELIMITER);
      }
    }
  }

  if (scanning && !paused) {
    returnToHome();
  }
}

// ==================== RESUME SCANNING FUNCTION ====================
void resumeScan() {
  int zLayers = (z_travel_mm * z_steps_per_mm) / z_steps_per_layer;
  int start_layer = paused_layer;
  int start_step = paused_step;

  if (start_layer == 0 && start_step == 0) {
    Serial.println("Resuming: Moving to top position...");
    moveToTop();
    delay(500);
  }

  int steps_per_point = steps_per_rev / theta_steps_per_rev;
  int remainder = steps_per_rev % theta_steps_per_rev;

  for (int layer = start_layer; layer < zLayers && scanning && !paused; layer++) {
    int step_start = (layer == start_layer) ? start_step : 0;
    for (int step = step_start; step < theta_steps_per_rev && scanning && !paused; step++) {
      if (!scanning || paused) break;

      int steps_this_point = steps_per_point;
      if (step < remainder) {
        steps_this_point++;
      }
      rotateMotorX(steps_this_point);
      if (!scanning || paused) break;

      current_theta_step = (current_theta_step + 1) % theta_steps_per_rev;

      int delay_remaining = scan_delay_ms;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = step;
        break;
      }

      delay(20);

      float distance = readVL53L0X();

      if (!paused) {
        float angle = (step % theta_steps_per_rev) * 360.0 / theta_steps_per_rev;
        Serial.print(layer);
        Serial.print(",");
        Serial.print(step);
        Serial.print(",");
        Serial.print(distance, 2);
        Serial.print(",");
        Serial.println(angle, 1);
      }
    }

    if (paused) {
      paused_layer = layer;
      paused_step = 0;
      break;
    }

    if (layer < zLayers - 1 && scanning && !paused) {
      if (!scanning || paused) break;

      digitalWrite(Z_DIR_PIN, HIGH);
      rotateMotorZ(z_steps_per_layer);
      digitalWrite(Z_DIR_PIN, LOW);
      if (!scanning || paused) break;

      int delay_remaining = 100;
      while (delay_remaining > 0 && scanning && !paused) {
        delay(10);
        delay_remaining -= 10;
      }
      if (!scanning || paused) {
        paused_layer = layer;
        paused_step = 0;
        break;
      }

      if (!paused) {
        Serial.println(DATA_DELIMITER);
      }
    }
  }
  if (scanning && !paused) {
    returnToHome();
  }
}

// ==================== CONFIGURATION FUNCTION ====================
void processConfigCommand(String command) {
  String params = command.substring(7);

  int commaIndex = 0;
  int intValues[6];
  float centerDist = 10.3;
  int stepsPerRev = DEFAULT_STEPS_PER_REV;

  for (int i = 0; i < 5; i++) {
    commaIndex = params.indexOf(',');
    if (commaIndex == -1) {
      intValues[i] = params.toInt();
      params = "";
      break;
    } else {
      intValues[i] = params.substring(0, commaIndex).toInt();
      params = params.substring(commaIndex + 1);
    }
  }

  if (params.length() > 0) {
    commaIndex = params.indexOf(',');
    if (commaIndex == -1) {
      centerDist = params.toFloat();
      params = "";
    } else {
      centerDist = params.substring(0, commaIndex).toFloat();
      params = params.substring(commaIndex + 1);
      if (params.length() > 0) {
        stepsPerRev = params.toInt();
      }
    }
  }

  if (intValues[0] < MIN_THETA_STEPS_PER_REV || intValues[0] > MAX_THETA_STEPS_PER_REV) {
    Serial.print("CONFIG_ERROR: theta_steps must be between ");
    Serial.print(MIN_THETA_STEPS_PER_REV);
    Serial.print(" and ");
    Serial.print(MAX_THETA_STEPS_PER_REV);
    Serial.println();
    return;
  }

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