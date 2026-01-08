#include <Arduino.h>

#include <optional>
#include <vector>

#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define pin_servo_tx 8
#define pin_servo_rx 18

#include "servo_bus_api.h"
#include "servo.h"
#include "sc_servo.h"
#include "sts_servo.h"



std::vector<uint8_t> scan_ids(uint32_t start_id=1, uint32_t end_id=255) {
  std::vector<uint8_t> found_servo_ids;
  for (uint32_t id = start_id; id<= end_id; ++id) {
    if (servo_bus.ping(id)) {
      found_servo_ids.push_back(id);
    }
  }
  return found_servo_ids;
}

void demonstrate_scan_ids(uint32_t start_id=1, uint32_t end_id=255) {
  Serial.printf("scanning for servo ids from %d to %d\n", start_id, end_id);
  auto servo_ids = scan_ids(start_id, end_id);
  for (uint32_t servo_id : servo_ids) {
    Serial.printf("found servo with id %d\n", servo_id);
  }
  Serial.println("done");
}

// Diagnostic function to check EEPROM lock status and ID configuration
void diagnose_servo(uint8_t servo_id, ServoBusApi::ServoType type) {
  Serial.printf("\n=== Servo ID %d Diagnostic ===\n", servo_id);

  servo_bus.set_servo_type(type);

  // Test if servo responds
  auto ping_result = servo_bus.ping(servo_id);
  if (!ping_result) {
    Serial.printf("ERROR: Servo ID %d does not respond to ping!\n", servo_id);
    Serial.println("Servo may be offline or have a different ID.\n");
    return;
  }
  Serial.printf("✓ Servo responds to ping\n");

  // Read current ID from EEPROM register
  auto id_value = servo_bus.read_byte(servo_id, ServoBusApi::Register::id);
  if (id_value) {
    Serial.printf("✓ ID register value: %d\n", *id_value);
  } else {
    Serial.printf("✗ Failed to read ID register\n");
  }

  // Read LOCK register status (use correct register for servo type)
  ServoBusApi::Register lock_reg = (type == ServoBusApi::ServoType::STS) ?
                                   ServoBusApi::Register::lock_sts :
                                   ServoBusApi::Register::lock_sc;
  uint8_t lock_reg_num = (type == ServoBusApi::ServoType::STS) ? 55 : 48;

  auto lock_value = servo_bus.read_byte(servo_id, lock_reg);
  if (lock_value) {
    Serial.printf("✓ LOCK register %d value: %d ", lock_reg_num, *lock_value);
    if (*lock_value == 0) {
      Serial.println("(UNLOCKED - EEPROM writes allowed)");
    } else if (*lock_value == 1) {
      Serial.println("(LOCKED - EEPROM writes blocked!)");
    } else {
      Serial.printf("(Unknown state: %d)\n", *lock_value);
    }
  } else {
    Serial.printf("✗ Failed to read LOCK register %d\n", lock_reg_num);
  }

  // Read servo version
  auto info = servo_bus.read_info(servo_id);
  if (info) {
    Serial.printf("✓ Firmware version: %d\n", info->version);
    Serial.printf("✓ Angle limits: min=%d, max=%d\n", info->min_angle, info->max_angle);
  } else {
    Serial.printf("✗ Failed to read servo info\n");
  }

  // Summary
  Serial.println("\n--- DIAGNOSIS ---");
  if (lock_value && *lock_value == 1) {
    Serial.println("⚠ WARNING: EEPROM is LOCKED!");
    Serial.println("  ID changes will NOT persist after power cycle.");
    Serial.println("  Use set_servo_id_permanent() to unlock, write, and lock.");
  } else if (lock_value && *lock_value == 0) {
    Serial.println("✓ EEPROM is unlocked - ID writes will persist.");
  }

  Serial.println("=================================\n");
}


void demonstrate_coordinated_moving() {
  Serial.println("=== Coordinated Servo Movement Test ===");

  // SC servos: IDs 2, 3, 4 (big-endian)
  // STS servo: ID 5 (little-endian)

  Serial.println("\n=== Reading servo ranges ===");

  std::vector<uint8_t> sc_ids = {2, 3, 4};
  std::vector<uint8_t> sts_ids = {5};
  std::vector<uint16_t> sc_mins, sc_maxs, sts_mins, sts_maxs;
  
  // Read SC servo ranges
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  Serial.println("SC servos:");
  for (auto id : sc_ids) {
    auto info = servo_bus.read_info(id);
    if (info) {
      Serial.printf("  Servo #%d: min=%d, max=%d\n", id, info->min_angle, info->max_angle);
      sc_mins.push_back(info->min_angle);
      sc_maxs.push_back(info->max_angle);
    } else {
      Serial.printf("  Servo #%d: Failed to read info\n", id);
      return;
    }
  }
  
  // Read STS servo ranges
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  Serial.println("STS servos:");
  for (auto id : sts_ids) {
    auto info = servo_bus.read_info(id);
    if (info) {
      Serial.printf("  Servo #%d: min=%d, max=%d\n", id, info->min_angle, info->max_angle);
      sts_mins.push_back(info->min_angle);
      sts_maxs.push_back(info->max_angle);
    } else {
      Serial.printf("  Servo #%d: Failed to read info\n", id);
      return;
    }
  }
  
  uint16_t target_time = 5000;  // 5 seconds target time
  
  Serial.println("\n=== Moving all servos from min to max in 5 seconds ===");
  
  // Calculate speeds based on distance to ensure same duration
  // Speed units are in position units per second
  std::vector<uint16_t> sc_speeds, sts_speeds;
  std::vector<uint16_t> sc_times, sts_times;
  
  for (size_t i = 0; i < sc_ids.size(); i++) {
    uint16_t distance = sc_maxs[i] - sc_mins[i];
    // Speed = distance / time (in seconds)
    uint16_t spd = (distance * 1000) / target_time;  // units per second
    sc_speeds.push_back(spd);
    sc_times.push_back(target_time);
    Serial.printf("  Servo #%d: distance=%d, speed=%d units/sec\n", sc_ids[i], distance, spd);
  }
  
  for (size_t i = 0; i < sts_ids.size(); i++) {
    uint16_t distance = sts_maxs[i] - sts_mins[i];
    uint16_t spd = (distance * 1000) / target_time;  // units per second
    sts_speeds.push_back(spd);
    sts_times.push_back(target_time);
    Serial.printf("  Servo #%d: distance=%d, speed=%d units/sec\n", sts_ids[i], distance, spd);
  }
  
  // Move SC servos to their max positions
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  servo_bus.sync_write_positions(sc_ids, sc_maxs, sc_times, sc_speeds);
  
  // Move STS servo to its max position
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  servo_bus.sync_write_positions(sts_ids, sts_maxs, sts_times, sts_speeds);
  
  Serial.println("\nMonitoring positions (reading 2x per second):");
  Serial.println("Time(ms)  | Servo #1 | Servo #2 | Servo #3 | Servo #4");
  Serial.println("----------|----------|----------|----------|----------");
  
  unsigned long start_time = millis();
  unsigned long duration = target_time + 1000;  // Add 1 second buffer
  
  while (millis() - start_time < duration) {
    unsigned long t = millis() - start_time;
    Serial.printf("%8lu  |", t);
    
    // Read SC servos
    servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
    for (auto id : sc_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    
    // Read STS servo
    servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
    for (auto id : sts_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d", *pos);
      } else {
        Serial.printf("     FAIL");
      }
    }
    Serial.println();
    
    delay(500);  // 2x per second
  }
  
  Serial.println("\n=== Moving back to min ===");
  
  // Move SC servos back to their min positions
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  servo_bus.sync_write_positions(sc_ids, sc_mins, sc_times, sc_speeds);
  
  // Move STS servo back to its min position
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  servo_bus.sync_write_positions(sts_ids, sts_mins, sts_times, sts_speeds);
  
  Serial.println("\nMonitoring positions (reading 2x per second):");
  Serial.println("Time(ms)  | Servo #1 | Servo #2 | Servo #3 | Servo #4");
  Serial.println("----------|----------|----------|----------|----------");
  
  start_time = millis();
  
  while (millis() - start_time < duration) {
    unsigned long t = millis() - start_time;
    Serial.printf("%8lu  |", t);
    
    // Read SC servos
    servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
    for (auto id : sc_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    
    // Read STS servo
    servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
    for (auto id : sts_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d", *pos);
      } else {
        Serial.printf("     FAIL");
      }
    }
    Serial.println();
    
    delay(500);  // 2x per second
  }
  
  Serial.println("\n=== Test Complete ===");
  Serial.println("\nNote: Had to call set_servo_type() multiple times and do separate");
  Serial.println("sync_write calls for SC vs STS servos. This is awkward with mixed types!");

}


void demonstrate_ping() {
  demonstrate_scan_ids(1, 10);
}

void demonstrate_coordinated_moving_2() {
  Serial.println("\n\n=== Coordinated Servo Movement Test (v2 - Using Servo Objects) ===");

  // Create servo objects - they remember their type!
  // SC servos: IDs 2, 3, 4 (big-endian)
  // STS servo: ID 5 (little-endian)
  auto servo2 = SCServo(&servo_bus, 2);
  auto servo3 = SCServo(&servo_bus, 3);
  auto servo4 = SCServo(&servo_bus, 4);
  auto servo5 = STSServo(&servo_bus, 5);

  // Store in array for easy iteration
  Servo* servos[] = {&servo2, &servo3, &servo4, &servo5};
  
  Serial.println("\n=== Reading servo info ===");
  for (auto* servo : servos) {
    if (servo->read_info()) {
      Serial.printf("Servo #%d: min=%d, max=%d, range=%d\n", 
                    servo->id(),
                    servo->min_encoder_angle(),
                    servo->max_encoder_angle(),
                    servo->encoder_angle_range());
    } else {
      Serial.printf("Servo #%d: Failed to read info\n", servo->id());
      return;
    }
  }
  
  Serial.println("\n=== Moving all servos to 100% (max) in 5 seconds ===");
  
  // Move all servos to max - each handles its own type switching!
  for (auto* servo : servos) {
    servo->move_to_percent(1.0f, 5000);
  }
  
  Serial.println("\nMonitoring positions (reading 2x per second):");
  Serial.println("Time(ms)  | Servo #1 | Servo #2 | Servo #3 | Servo #4");
  Serial.println("----------|----------|----------|----------|----------");
  
  unsigned long start_time = millis();
  unsigned long duration = 6000;
  
  while (millis() - start_time < duration) {
    unsigned long t = millis() - start_time;
    Serial.printf("%8lu  |", t);
    
    for (auto* servo : servos) {
      auto pos = servo->read_encoder_angle();
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    Serial.println();
    
    delay(500);
  }
  
  Serial.println("\n=== Moving all servos to 0% (min) in 5 seconds ===");
  
  // Move all servos to min
  for (auto* servo : servos) {
    servo->move_to_percent(0.0f, 5000);
  }
  
  Serial.println("\nMonitoring positions (reading 2x per second):");
  Serial.println("Time(ms)  | Servo #1 | Servo #2 | Servo #3 | Servo #4");
  Serial.println("----------|----------|----------|----------|----------");
  
  start_time = millis();
  
  while (millis() - start_time < duration) {
    unsigned long t = millis() - start_time;
    Serial.printf("%8lu  |", t);
    
    for (auto* servo : servos) {
      auto pos = servo->read_encoder_angle();
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    Serial.println();
    
    delay(500);
  }
  
  Serial.println("\n=== Moving to 50% in 3 seconds ===");
  
  for (auto* servo : servos) {
    servo->move_to_percent(0.5f, 3000);
  }
  
  delay(3500);
  
  Serial.println("\nFinal positions:");
  for (auto* servo : servos) {
    auto pos = servo->read_encoder_angle();
    if (pos) {
      Serial.printf("Servo #%d: %d (%.1f%%)\n", 
                    servo->id(), 
                    *pos,
                    100.0f * (*pos - servo->min_encoder_angle()) / servo->encoder_angle_range());
    }
  }
  
  Serial.println("\n=== Test Complete ===");
  Serial.println("\nMuch cleaner! No manual set_servo_type() calls!");
  Serial.println("Each servo object handles its own type internally.");
}

void demonstrate_sts_features() {
  Serial.println("\n\n=== STS Servo Special Features Demo ===");

  auto sts_servo = STSServo(&servo_bus, 5);
  
  Serial.println("\n=== Reading STS servo info ===");
  if (!sts_servo.read_info()) {
    Serial.println("Failed to read STS servo info");
    return;
  }
  
  Serial.printf("STS Servo #%d: min=%d, max=%d, range=%d\n",
                sts_servo.id(),
                sts_servo.min_encoder_angle(),
                sts_servo.max_encoder_angle(),
                sts_servo.encoder_angle_range());
  
  // Move to center position first
  Serial.println("\n=== Moving to center position ===");
  sts_servo.move_to_percent(0.5f, 2000);
  delay(2500);
  
  auto center = sts_servo.read_encoder_angle();
  if (!center) {
    Serial.println("Failed to read center position");
    return;
  }
  Serial.printf("Center position: %d\n", *center);
  
  // Hardware acceleration test - demonstrate STS ACC register
  Serial.println("\n=== Hardware Acceleration Test - STS ACC Register ===");
  Serial.println("This test demonstrates true hardware acceleration using register 41.");
  Serial.println("Position deltas [Δ] show velocity - watch for ramp-up/ramp-down patterns!\n");

  uint16_t range = sts_servo.max_encoder_angle() - sts_servo.min_encoder_angle();
  uint16_t speed = (range * 1000) / 3000;  // ~3 second movement at target velocity

  Serial.printf("Servo range: %d units, Target speed: %d units/sec\n\n", range, speed);

  // Move to start position (0%) first
  Serial.println("Moving to start position (0%)...");
  sts_servo.move_to_percent(0.0f, 2000);
  delay(2500);

  // Test 1: ACC=0 (No acceleration)
  Serial.println("\nMovement 1: ACC=0 (No acceleration - abrupt start/stop)");
  Serial.printf("Moving 0%% → 100%% with speed=%d\n", speed);
  sts_servo.move_to_percent_with_accel(1.0f, speed, 0);
  delay(500);

  unsigned long start = millis();
  int last_pos = -1;
  while (millis() - start < 4000) {
    auto pos = sts_servo.read_encoder_angle();
    if (pos) {
      int delta = (last_pos >= 0) ? (*pos - last_pos) : 0;
      float percent = 100.0f * (*pos - sts_servo.min_encoder_angle()) / sts_servo.encoder_angle_range();
      Serial.printf("  t=%4lums: pos=%4d (%5.1f%%)  [Δ=%4d]\n",
                    millis() - start, *pos, percent, delta);
      last_pos = *pos;
    }
    delay(150);
  }

  // Return to start for next test
  Serial.println("Returning to 0%...");
  sts_servo.move_to_percent(0.0f, 2000);
  delay(2500);

  // Test 2: ACC=50 (Moderate acceleration)
  Serial.println("\nMovement 2: ACC=50 (Moderate acceleration)");
  Serial.printf("Moving 0%% → 100%% with speed=%d\n", speed);
  sts_servo.move_to_percent_with_accel(1.0f, speed, 50);
  delay(500);

  start = millis();
  last_pos = -1;
  while (millis() - start < 5000) {
    auto pos = sts_servo.read_encoder_angle();
    if (pos) {
      int delta = (last_pos >= 0) ? (*pos - last_pos) : 0;
      float percent = 100.0f * (*pos - sts_servo.min_encoder_angle()) / sts_servo.encoder_angle_range();
      Serial.printf("  t=%4lums: pos=%4d (%5.1f%%)  [Δ=%4d]\n",
                    millis() - start, *pos, percent, delta);
      last_pos = *pos;
    }
    delay(150);
  }

  // Return to start for next test
  Serial.println("Returning to 0%...");
  sts_servo.move_to_percent(0.0f, 2000);
  delay(2500);

  // Test 3: ACC=200 (High acceleration)
  Serial.println("\nMovement 3: ACC=200 (High acceleration - very smooth)");
  Serial.printf("Moving 0%% → 100%% with speed=%d\n", speed);
  sts_servo.move_to_percent_with_accel(1.0f, speed, 200);
  delay(500);

  start = millis();
  last_pos = -1;
  while (millis() - start < 6000) {
    auto pos = sts_servo.read_encoder_angle();
    if (pos) {
      int delta = (last_pos >= 0) ? (*pos - last_pos) : 0;
      float percent = 100.0f * (*pos - sts_servo.min_encoder_angle()) / sts_servo.encoder_angle_range();
      Serial.printf("  t=%4lums: pos=%4d (%5.1f%%)  [Δ=%4d]\n",
                    millis() - start, *pos, percent, delta);
      last_pos = *pos;
    }
    delay(150);
  }

  Serial.println("\n=== Acceleration Test Complete ===");
  Serial.println("Notice the difference in position deltas:");
  Serial.println("  ACC=0:   Immediate large deltas (abrupt speed change)");
  Serial.println("  ACC=50:  Moderate ramp-up/ramp-down");
  Serial.println("  ACC=200: Gradual ramp-up/ramp-down (smooth velocity curve)");

  // Continuous rotation - STS servos can rotate 360+ degrees in wheel mode!
  Serial.println("\n=== Continuous Rotation (Wheel Mode) ===");
  Serial.println("STS servos support wheel mode for continuous rotation!");
  Serial.println("Setting servo to wheel mode (min=0, max=0)...");
  
  // Save original limits
  uint16_t saved_min = sts_servo.min_encoder_angle();
  uint16_t saved_max = sts_servo.max_encoder_angle();
  
  // Enable wheel mode
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  if (!servo_bus.enable_wheel_mode(sts_servo.id())) {
    Serial.println("Failed to enable wheel mode!");
    return;
  }
  
  Serial.println("Wheel mode enabled! Now controlling velocity instead of position.");
  Serial.println("\nIn wheel mode:");
  Serial.println("  speed > 0: clockwise (higher = faster)");
  Serial.println("  speed = 0: stopped");
  Serial.println("  speed < 0: counter-clockwise (more negative = faster)");

  Serial.println("\nRotating clockwise at different speeds...");
  int speeds[] = {100, 200, 400, 700, 1000};  // Positive values for CW direction
  const char* labels[] = {"Slow", "Medium-Slow", "Medium", "Medium-Fast", "Fast"};

  for (int i = 0; i < 5; i++) {
    Serial.printf("\n%s CW rotation (speed=%d):\n", labels[i], speeds[i]);
    servo_bus.set_wheel_velocity(sts_servo.id(), speeds[i]);
    
    // Monitor position while rotating
    unsigned long start = millis();
    int last_pos = -1;
    int rotation_count = 0;
    
    for (int j = 0; j < 8; j++) {  // Check 8 times over 2 seconds
      delay(250);
      auto pos = servo_bus.read_position(sts_servo.id());
      if (pos) {
        // Detect wrap-around (encoder jumps from high to low = forward rotation)
        if (last_pos != -1) {
          int delta = *pos - last_pos;
          if (delta < -2000) {  // Wrapped forward (4095 -> 0)
            rotation_count++;
          } else if (delta > 2000) {  // Wrapped backward (0 -> 4095)
            rotation_count--;
          }
        }
        Serial.printf("  t=%4lums: pos=%5d (rotations=%d)\n", 
                      millis() - start, *pos, rotation_count);
        last_pos = *pos;
      }
    }
  }
  
  Serial.println("\nRotating counter-clockwise...");
  int ccw_speeds[] = {-100, -200, -400, -700, -1000};  // Negative values for CCW direction

  for (int i = 0; i < 5; i++) {
    Serial.printf("\n%s CCW rotation (speed=%d):\n", labels[i], ccw_speeds[i]);
    servo_bus.set_wheel_velocity(sts_servo.id(), ccw_speeds[i]);
    
    unsigned long start = millis();
    int last_pos = -1;
    int rotation_count = 0;
    
    for (int j = 0; j < 8; j++) {
      delay(250);
      auto pos = servo_bus.read_position(sts_servo.id());
      if (pos) {
        // Detect wrap-around (encoder jumps from low to high = backward rotation)
        if (last_pos != -1) {
          int delta = *pos - last_pos;
          if (delta < -2000) {  // Wrapped forward (4095 -> 0)
            rotation_count--;
          } else if (delta > 2000) {  // Wrapped backward (0 -> 4095)
            rotation_count++;
          }
        }
        Serial.printf("  t=%4lums: pos=%5d (rotations=%d)\n", 
                      millis() - start, *pos, rotation_count);
        last_pos = *pos;
      }
    }
  }
  
  // Stop the motor
  Serial.println("\nStopping rotation (speed=0)...");
  servo_bus.set_wheel_velocity(sts_servo.id(), 0);
  delay(500);
  
  auto stopped_pos = servo_bus.read_position(sts_servo.id());
  if (stopped_pos) {
    Serial.printf("Stopped at position: %d\n", *stopped_pos);
  }
  
  // Restore position mode
  Serial.println("\nRestoring position mode...");
  if (!servo_bus.enable_position_mode(sts_servo.id(), saved_min, saved_max)) {
    Serial.println("Failed to restore position mode!");
    return;
  }
  
  // Re-read servo info to update cached limits
  sts_servo.read_info();
  
  Serial.println("Position mode restored!");
  Serial.println("\nThis is unique to STS servos - SC servos cannot do wheel mode!");
  
  // Return to center
  Serial.println("\n=== Returning to center ===");
  sts_servo.move_to_percent(0.5f, 2000);
  delay(2500);
  
  auto final_pos = sts_servo.read_encoder_angle();
  if (final_pos) {
    Serial.printf("Final position: %d\n", *final_pos);
  }
  
  Serial.println("\n=== STS Features Demo Complete ===");
  Serial.println("STS servos excel at smooth, continuous motion patterns!");
}

void demonstrate_wheel_mode() {
  Serial.println("=== STS Servo Wheel Mode Demo ===");

  auto sts_servo = STSServo(&servo_bus, 5);
  
  Serial.println("\n=== Reading STS servo info ===");
  if (!sts_servo.read_info()) {
    Serial.println("Failed to read STS servo info");
    return;
  }
  
  Serial.printf("STS Servo #%d: min=%d, max=%d, range=%d\n",
                sts_servo.id(),
                sts_servo.min_encoder_angle(),
                sts_servo.max_encoder_angle(),
                sts_servo.encoder_angle_range());
  
  Serial.println("\n=== Continuous Rotation (Wheel Mode) ===");
  Serial.println("STS servos support wheel mode for continuous rotation!");
  Serial.println("Enabling wheel mode...");
  
  // Enable wheel mode - no need for manual type switching!
  if (!sts_servo.enable_wheel_mode()) {
    Serial.println("Failed to enable wheel mode!");
    return;
  }
  
  Serial.println("Wheel mode enabled! Now controlling velocity instead of position.");
  Serial.println("\nIn wheel mode:");
  Serial.println("  speed > 0: clockwise (higher = faster)");
  Serial.println("  speed = 0: stopped");
  Serial.println("  speed < 0: counter-clockwise (more negative = faster)");

  Serial.println("\nRotating clockwise at different speeds...");
  int speeds[] = {100, 200, 400, 700, 1000};  // Positive values for CW direction
  const char* labels[] = {"Slow", "Medium-Slow", "Medium", "Medium-Fast", "Fast"};

  for (int i = 0; i < 5; i++) {
    Serial.printf("\n%s CW rotation (speed=%d):\n", labels[i], speeds[i]);
    sts_servo.set_wheel_velocity(speeds[i]);
    
    // Monitor position while rotating
    unsigned long start = millis();
    int last_pos = -1;
    int rotation_count = 0;
    
    for (int j = 0; j < 8; j++) {  // Check 8 times over 2 seconds
      delay(250);
      auto pos = sts_servo.read_encoder_angle();
      if (pos) {
        // Detect wrap-around (encoder jumps from high to low = forward rotation)
        if (last_pos != -1) {
          int delta = *pos - last_pos;
          if (delta < -2000) {  // Wrapped forward (4095 -> 0)
            rotation_count++;
          } else if (delta > 2000) {  // Wrapped backward (0 -> 4095)
            rotation_count--;
          }
        }
        Serial.printf("  t=%4lums: pos=%5d (rotations=%d)\n", 
                      millis() - start, *pos, rotation_count);
        last_pos = *pos;
      }
    }
  }
  
  Serial.println("\nRotating counter-clockwise...");
  int ccw_speeds[] = {-100, -200, -400, -700, -1000};  // Negative values for CCW direction

  for (int i = 0; i < 5; i++) {
    Serial.printf("\n%s CCW rotation (speed=%d):\n", labels[i], ccw_speeds[i]);
    sts_servo.set_wheel_velocity(ccw_speeds[i]);
    
    unsigned long start = millis();
    int last_pos = -1;
    int rotation_count = 0;
    
    for (int j = 0; j < 8; j++) {
      delay(250);
      auto pos = sts_servo.read_encoder_angle();
      if (pos) {
        // Detect wrap-around (encoder jumps from low to high = backward rotation)
        if (last_pos != -1) {
          int delta = *pos - last_pos;
          if (delta < -2000) {  // Wrapped forward (4095 -> 0)
            rotation_count--;
          } else if (delta > 2000) {  // Wrapped backward (0 -> 4095)
            rotation_count++;
          }
        }
        Serial.printf("  t=%4lums: pos=%5d (rotations=%d)\n", 
                      millis() - start, *pos, rotation_count);
        last_pos = *pos;
      }
    }
  }
  
  // Stop the motor
  Serial.println("\nStopping rotation (speed=0)...");
  sts_servo.set_wheel_velocity(0);
  delay(500);
  
  auto stopped_pos = sts_servo.read_encoder_angle();
  if (stopped_pos) {
    Serial.printf("Stopped at position: %d\n", *stopped_pos);
  }
  
  // Restore position mode
  Serial.println("\nRestoring position mode...");
  if (!sts_servo.enable_position_mode()) {
    Serial.println("Failed to restore position mode!");
    return;
  }
  
  Serial.println("Position mode restored!");
  
  // Return to center
  Serial.println("\n=== Returning to center ===");
  sts_servo.move_to_percent(0.5f, 2000);
  delay(2500);
  
  auto final_pos = sts_servo.read_encoder_angle();
  if (final_pos) {
    Serial.printf("Final position: %d\n", *final_pos);
  }
  
  Serial.println("\n=== Wheel Mode Demo Complete ===");
  Serial.println("\nThis is unique to STS servos - SC servos cannot do wheel mode!");
  Serial.println("Wheel mode enables continuous rotation for wheels, turntables, etc.");
}

void demonstrate_pwm_mode() {
  Serial.println("=== PWM Mode Demo - All Servos ===");
  Serial.println("PWM mode works on both SC and STS servos!");

  // Create servo objects
  auto servo4 = SCServo(&servo_bus, 4);
  auto servo5 = STSServo(&servo_bus, 5);

  Servo* servos[] = {&servo4, &servo5};
  
  Serial.println("\n=== Reading servo info ===");
  for (auto* servo : servos) {
    if (!servo->read_info()) {
      Serial.printf("Servo #%d: Failed to read info\n", servo->id());
      return;
    }
    Serial.printf("Servo #%d: Ready\n", servo->id());
  }
  
  Serial.println("\n=== Enabling PWM mode on all servos ===");
  for (auto* servo : servos) {
    if (servo->enable_pwm_mode()) {
      Serial.printf("Servo #%d: PWM mode enabled\n", servo->id());
    } else {
      Serial.printf("Servo #%d: Failed to enable PWM mode\n", servo->id());
    }
  }
  
  Serial.println("\n=== Testing CW rotation at different speeds ===");
  int speeds[] = {100, 300, 500, 700};
  const char* labels[] = {"Slow", "Medium", "Fast", "Very Fast"};
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("\n%s CW rotation (PWM=%d) - 2 seconds:\n", labels[i], speeds[i]);
    
    for (auto* servo : servos) {
      servo->set_pwm_speed(speeds[i]);
    }
    
    unsigned long start = millis();
    while (millis() - start < 2000) {
      Serial.printf("t=%4lums: ", millis() - start);
      for (auto* servo : servos) {
        auto pos = servo->read_encoder_angle();
        if (pos) {
          Serial.printf("S%d:%4d  ", servo->id(), *pos);
        }
      }
      Serial.println();
      delay(250);
    }
  }
  
  Serial.println("\n=== Testing CCW rotation at different speeds ===");
  int ccw_speeds[] = {-100, -300, -500, -700};
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("\n%s CCW rotation (PWM=%d) - 2 seconds:\n", labels[i], ccw_speeds[i]);
    
    for (auto* servo : servos) {
      servo->set_pwm_speed(ccw_speeds[i]);
    }
    
    unsigned long start = millis();
    while (millis() - start < 2000) {
      Serial.printf("t=%4lums: ", millis() - start);
      for (auto* servo : servos) {
        auto pos = servo->read_encoder_angle();
        if (pos) {
          Serial.printf("S%d:%4d  ", servo->id(), *pos);
        }
      }
      Serial.println();
      delay(250);
    }
  }
  
  Serial.println("\n=== Stopping all servos ===");
  for (auto* servo : servos) {
    servo->set_pwm_speed(0);  // Stop (PWM=0)
  }
  delay(500);
  
  Serial.println("\nFinal positions:");
  for (auto* servo : servos) {
    auto pos = servo->read_encoder_angle();
    if (pos) {
      Serial.printf("Servo #%d: %d\n", servo->id(), *pos);
    }
  }
  
  Serial.println("\n=== PWM Mode Demo Complete ===");
  Serial.println("PWM mode works on all servo types - SC and STS!");
}

// Emergency recovery function to clear stuck servo output
void emergency_servo_reset() {
  Serial.println("\n=== EMERGENCY SERVO RESET ===");
  Serial.println("Servo appears stuck in continuous broadcast mode!");
  
  // Step 1: Analyze what's coming in (brief check)
  Serial.println("Analyzing incoming packets (1 second)...");
  unsigned long start = millis();
  int packet_count = 0;
  uint8_t buffer[100];
  int buf_pos = 0;
  
  while (millis() - start < 1000) {
    if (Serial1.available()) {
      uint8_t b = Serial1.read();
      buffer[buf_pos++] = b;
      Serial.printf("%02X", b);
      
      // Look for packet headers (FF FF)
      if (buf_pos >= 2 && buffer[buf_pos-2] == 0xFF && buffer[buf_pos-1] == 0xFF) {
        Serial.print(" <-HDR ");
        packet_count++;
      }
      
      if (buf_pos >= 100) {
        Serial.println("\n[Buffer full]");
        buf_pos = 0;
      }
    }
  }
  Serial.printf("\n\nDetected %d packet headers\n", packet_count);
  
  // Step 2: Clear buffer aggressively
  Serial.println("Draining buffer...");
  int cleared = 0;
  start = millis();
  while (Serial1.available() && millis() - start < 500) {
    Serial1.read();
    cleared++;
  }
  Serial.printf("Cleared %d bytes\n", cleared);
  
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  
  // Step 3: Try to reset mode on ID 252 (appears to be broadcasting)
  Serial.println("Attempting to reset servo ID 252 mode...");
  uint8_t mode_params[2];
  mode_params[0] = 33;  // SMS_STS_MODE register
  mode_params[1] = 0;   // Set to normal servo mode (not wheel mode)
  servo_bus.send_command(252, ServoBusApi::Instruction::write, mode_params, 2);
  delay(100);
  
  // Step 4: Clear buffer again
  cleared = 0;
  start = millis();
  while (Serial1.available() && millis() - start < 500) {
    Serial1.read();
    cleared++;
  }
  if (cleared > 0) {
    Serial.printf("Cleared %d more bytes\n", cleared);
  }
  
  // Step 5: Disable torque on broadcast ID
  Serial.println("Disabling torque (broadcast)...");
  uint8_t torque_params[2];
  torque_params[0] = ServoBusApi::to_byte(ServoBusApi::Register::torque_enable);
  torque_params[1] = 0;  // Disable
  servo_bus.send_command(0xFE, ServoBusApi::Instruction::write, torque_params, 2);
  delay(100);
  
  // Step 6: Try to set torque limit to 0 (ID 252)
  Serial.println("Setting torque limit to 0...");
  uint8_t torque_limit_params[3];
  torque_limit_params[0] = 48;  // SMS_STS_TORQUE_LIMIT_L
  torque_limit_params[1] = 0;   // LOW byte
  torque_limit_params[2] = 0;   // HIGH byte
  servo_bus.send_command(252, ServoBusApi::Instruction::write, torque_limit_params, 3);
  delay(100);
  
  // Step 7: Try specific servo IDs (1-4)
  Serial.println("Trying individual servo IDs 1-4...");
  for (uint8_t id = 1; id <= 4; id++) {
    // Set mode to 0 (servo mode)
    mode_params[0] = 33;  // MODE
    mode_params[1] = 0;   // Normal mode
    servo_bus.send_command(id, ServoBusApi::Instruction::write, mode_params, 2);
    delay(30);
    
    // Disable torque
    torque_params[0] = 40;  // TORQUE_ENABLE
    torque_params[1] = 0;   // Disable
    servo_bus.send_command(id, ServoBusApi::Instruction::write, torque_params, 2);
    delay(30);
  }
  
  // Step 7: Final check
  delay(500);
  cleared = 0;
  start = millis();
  while (Serial1.available() && millis() - start < 1000) {
    Serial1.read();
    cleared++;
  }
  
  if (cleared == 0) {
    Serial.println("✓ SUCCESS: Buffer is clear - servo stopped!");
  } else {
    Serial.printf("⚠ WARNING: Still receiving %d bytes - POWER CYCLE REQUIRED\n", cleared);
  }
  
  Serial.println("=====================================\n");
}

// Helper function to wait for user to press Enter
void wait_for_enter(const char* prompt = nullptr) {
  if (prompt) {
    Serial.println(prompt);
  }
  Serial.println("\n>>> Press ENTER to continue...");
  Serial.print(">>> ");

  // Clear any existing input
  while (Serial.available()) {
    Serial.read();
  }

  // Wait for Enter key (newline) and echo characters
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        // Clear any remaining input
        delay(10);
        while (Serial.available()) {
          Serial.read();
        }
        Serial.println();
        return;
      }
      // Echo other characters
      Serial.print(c);
    }
    delay(10);
  }
}

void demonstrate_voltage() {
  SCServo sc_servo(&servo_bus, 4);
  STSServo sts_servo(&servo_bus, 5);

  Servo* servos[] = {&sc_servo, &sts_servo};

  Serial.printf("reading voltages\n");

  for (Servo * servo : servos ) {
    auto voltage = servo->read_voltage();
    if(voltage) {
      Serial.printf("Servo %d voltage %.1f\n", servo->id(), *voltage);
    } else {
      Serial.printf("Failed reading voltage for servo %d", servo->id());
    }
  }

  Serial.printf("done\n");

}


void demonstrate_infer_servo_type() {
  auto servo_ids = scan_ids();

  Serial.printf("Inferring servo types for %d servos\n", servo_ids.size());

  for (auto servo_id : servo_ids) {
    Serial.printf("Servo %d: ", servo_id);
    
    auto type = Servo::infer_servo_type(&servo_bus, servo_id);
    
    if (!type) {
      Serial.printf("UNKNOWN - could not determine type\n");
      continue;
    }
    
    if (*type == ServoBusApi::ServoType::STS) {
      Serial.printf("STS\n");
    } else {
      Serial.printf("SC\n");
    }
  }
  
  Serial.println("Done inferring servo types");
}

void demonstrate_temperature() {
  auto servo_ids = scan_ids();

  Serial.printf("reading temperatures\n");

  for (auto servo_id : servo_ids ) {
    SCServo servo(&servo_bus, servo_id);
    auto temperature = servo.read_temperature();
    if (!temperature) {
      Serial.printf("Failed reading temperature for servo %d", servo.id());
    }

    auto voltage = servo.read_voltage();
    if (!voltage) {
      Serial.printf("Failed reading voltage for servo %d", servo.id());
    }
    if(temperature) {
      Serial.printf("Servo %d temperature %f voltage: %f\n", servo.id(), *temperature, *voltage);
    } else {
    }
  }

  Serial.printf("done\n");

}




// Helper function to ask yes/no question
bool ask_yes_no(const char* question) {
  Serial.println(question);
  Serial.println(">>> Type 'y' or 'n' and press ENTER:");
  Serial.print(">>> ");

  // Clear any existing input
  while (Serial.available()) {
    Serial.read();
  }

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      Serial.print(c); // Echo

      // Clear rest of line
      delay(10);
      while (Serial.available()) {
        char extra = Serial.read();
        Serial.print(extra); // Echo everything
      }

      if (c == 'y' || c == 'Y') {
        Serial.println();
        return true;
      } else if (c == 'n' || c == 'N') {
        Serial.println();
        return false;
      } else {
        Serial.println("\nPlease type 'y' or 'n':");
        Serial.print(">>> ");
      }
    }
    delay(10);
  }
}

void restore_sts_eprom() {
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       STS Servo EPROM Restoration                           ║");
  Serial.println("║  This will write angle limits 0-4095 to EPROM               ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  if (!ask_yes_no("This will permanently restore the STS servo #5 angle limits.\nContinue?")) {
    Serial.println("Cancelled.");
    return;
  }

  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  
  // Step 1: Unlock EPROM (LOCK register = 0)
  Serial.println("\n═══ STEP 1: Unlocking EPROM ═══");
  uint8_t unlock_params[2];
  unlock_params[0] = ServoBusApi::to_byte(ServoBusApi::Register::lock_sts);  // Register 55
  unlock_params[1] = 0;  // Unlock
  
  if (servo_bus.send_command(5, ServoBusApi::Instruction::write, unlock_params, 2)) {
    uint8_t unlock_response[6];
    if (servo_bus.read_response(unlock_response, 6)) {
      Serial.println("✓ EPROM unlocked");
    } else {
      Serial.println("✗ No response from unlock command");
      return;
    }
  } else {
    Serial.println("✗ Failed to send unlock command");
    return;
  }
  
  delay(100);
  
  // Step 2: Write angle limits (0-4095) to registers 9-12
  Serial.println("\n═══ STEP 2: Writing angle limits (0-4095) ═══");
  uint8_t limits_params[5];
  limits_params[0] = ServoBusApi::to_byte(ServoBusApi::Register::min_angle_limit_l);  // Register 9
  limits_params[1] = 0x00;  // min LOW byte = 0
  limits_params[2] = 0x00;  // min HIGH byte = 0
  limits_params[3] = 0xFF;  // max LOW byte = 255
  limits_params[4] = 0x0F;  // max HIGH byte = 15 (0x0FFF = 4095)
  
  if (servo_bus.send_command(5, ServoBusApi::Instruction::write, limits_params, 5)) {
    uint8_t limits_response[6];
    if (servo_bus.read_response(limits_response, 6)) {
      Serial.println("✓ Angle limits written");
    } else {
      Serial.println("✗ No response from write command");
      return;
    }
  } else {
    Serial.println("✗ Failed to send write command");
    return;
  }
  
  delay(100);
  
  // Step 3: Lock EPROM (LOCK register = 1)
  Serial.println("\n═══ STEP 3: Locking EPROM ═══");
  uint8_t lock_params[2];
  lock_params[0] = ServoBusApi::to_byte(ServoBusApi::Register::lock_sts);  // Register 55
  lock_params[1] = 1;  // Lock
  
  if (servo_bus.send_command(5, ServoBusApi::Instruction::write, lock_params, 2)) {
    uint8_t lock_response[6];
    if (servo_bus.read_response(lock_response, 6)) {
      Serial.println("✓ EPROM locked");
    } else {
      Serial.println("✗ No response from lock command");
      return;
    }
  } else {
    Serial.println("✗ Failed to send lock command");
    return;
  }
  
  delay(500);
  
  // Step 4: Verify by reading back
  Serial.println("\n═══ STEP 4: Verifying ═══");
  auto sts_servo = STSServo(&servo_bus, 5);
  if (sts_servo.read_info()) {
    Serial.printf("✓ STS Servo #5 NOW: min=%d, max=%d, range=%d\n",
                  sts_servo.min_encoder_angle(),
                  sts_servo.max_encoder_angle(),
                  sts_servo.encoder_angle_range());
    
    if (sts_servo.min_encoder_angle() == 0 && sts_servo.max_encoder_angle() == 4095) {
      Serial.println("\n✓✓✓ SUCCESS! EPROM restored correctly! ✓✓✓");
    } else {
      Serial.println("\n⚠ EPROM values written but don't match expected 0-4095");
    }
  } else {
    Serial.println("✗ Failed to read servo info for verification");
  }
  
  Serial.println("\nPress ENTER to continue...");
  wait_for_enter();
}

void diagnose_eprom_registers() {
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       EPROM Register Diagnostic                              ║");
  Serial.println("║  Reading raw register values from both servos                ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  // Helper to print raw response bytes
  auto print_response = [](const char* label, uint8_t* response, int size) {
    Serial.printf("%s: ", label);
    for (int i = 0; i < size; i++) {
      Serial.printf("%02X ", response[i]);
    }
    Serial.println();
  };

  // ═══ SC SERVO #4 ═══
  Serial.println("═══ SC SERVO #4 ═══");
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  
  // Read min angle limit (registers 9-10, 2 bytes)
  Serial.println("\nReading MIN_ANGLE_LIMIT (registers 9-10, 2 bytes)...");
  uint8_t sc_min_params[] = {9, 2};
  if (servo_bus.send_command(4, ServoBusApi::Instruction::read, sc_min_params, 2)) {
    uint8_t sc_min_response[8];
    if (servo_bus.read_response(sc_min_response, 8)) {
      print_response("  Raw response", sc_min_response, 8);
      uint16_t min_val = sc_min_response[5] | (sc_min_response[6] << 8);
      Serial.printf("  Parsed value (BE): %d (0x%04X)\n", min_val, min_val);
    } else {
      Serial.println("  ✗ Failed to read response");
    }
  } else {
    Serial.println("  ✗ Failed to send command");
  }

  delay(50);

  // Read max angle limit (registers 11-12, 2 bytes)
  Serial.println("\nReading MAX_ANGLE_LIMIT (registers 11-12, 2 bytes)...");
  uint8_t sc_max_params[] = {11, 2};
  if (servo_bus.send_command(4, ServoBusApi::Instruction::read, sc_max_params, 2)) {
    uint8_t sc_max_response[8];
    if (servo_bus.read_response(sc_max_response, 8)) {
      print_response("  Raw response", sc_max_response, 8);
      uint16_t max_val = sc_max_response[5] | (sc_max_response[6] << 8);
      Serial.printf("  Parsed value (BE): %d (0x%04X)\n", max_val, max_val);
    } else {
      Serial.println("  ✗ Failed to read response");
    }
  } else {
    Serial.println("  ✗ Failed to send command");
  }

  delay(50);

  // Use read_info() method
  Serial.println("\nUsing read_info() method:");
  auto sc_info = servo_bus.read_info(4);
  if (sc_info.has_value()) {
    Serial.printf("  ✓ Version: %d, Min: %d, Max: %d\n", 
                  sc_info->version, sc_info->min_angle, sc_info->max_angle);
  } else {
    Serial.println("  ✗ read_info() failed");
  }

  delay(500);

  // ═══ STS SERVO #5 ═══
  Serial.println("\n═══ STS SERVO #5 ═══");
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  
  // Read MODE register (register 33, 1 byte)
  Serial.println("\nReading MODE register (register 33, 1 byte)...");
  uint8_t sts_mode_params[] = {33, 1};
  if (servo_bus.send_command(5, ServoBusApi::Instruction::read, sts_mode_params, 2)) {
    uint8_t sts_mode_response[7];
    if (servo_bus.read_response(sts_mode_response, 7)) {
      print_response("  Raw response", sts_mode_response, 7);
      uint8_t mode_val = sts_mode_response[5];
      Serial.printf("  MODE value: %d (", mode_val);
      if (mode_val == 0) Serial.print("Position");
      else if (mode_val == 1) Serial.print("Wheel");
      else Serial.print("Unknown");
      Serial.println(")");
    } else {
      Serial.println("  ✗ Failed to read response");
    }
  } else {
    Serial.println("  ✗ Failed to send command");
  }

  delay(50);

  // Read min angle limit (registers 9-10, 2 bytes)
  Serial.println("\nReading MIN_ANGLE_LIMIT (registers 9-10, 2 bytes)...");
  uint8_t sts_min_params[] = {9, 2};
  if (servo_bus.send_command(5, ServoBusApi::Instruction::read, sts_min_params, 2)) {
    uint8_t sts_min_response[8];
    if (servo_bus.read_response(sts_min_response, 8)) {
      print_response("  Raw response", sts_min_response, 8);
      uint16_t min_val = sts_min_response[5] | (sts_min_response[6] << 8);
      Serial.printf("  Parsed value (LE): %d (0x%04X)\n", min_val, min_val);
    } else {
      Serial.println("  ✗ Failed to read response");
    }
  } else {
    Serial.println("  ✗ Failed to send command");
  }

  delay(50);

  // Read max angle limit (registers 11-12, 2 bytes)
  Serial.println("\nReading MAX_ANGLE_LIMIT (registers 11-12, 2 bytes)...");
  uint8_t sts_max_params[] = {11, 2};
  if (servo_bus.send_command(5, ServoBusApi::Instruction::read, sts_max_params, 2)) {
    uint8_t sts_max_response[8];
    if (servo_bus.read_response(sts_max_response, 8)) {
      print_response("  Raw response", sts_max_response, 8);
      uint16_t max_val = sts_max_response[5] | (sts_max_response[6] << 8);
      Serial.printf("  Parsed value (LE): %d (0x%04X)\n", max_val, max_val);
    } else {
      Serial.println("  ✗ Failed to read response");
    }
  } else {
    Serial.println("  ✗ Failed to send command");
  }

  delay(50);

  // Use read_info() method
  Serial.println("\nUsing read_info() method:");
  auto sts_info = servo_bus.read_info(5);
  if (sts_info.has_value()) {
    Serial.printf("  ✓ Version: %d, Min: %d, Max: %d\n", 
                  sts_info->version, sts_info->min_angle, sts_info->max_angle);
  } else {
    Serial.println("  ✗ read_info() failed");
  }

  Serial.println("\n═══ DIAGNOSTIC COMPLETE ═══");
  Serial.println("If STS servo shows min=0, max=0, its EPROM may need restoration.");
  Serial.println("\nPress ENTER to continue...\n");
  wait_for_enter();
}

void test_speed_and_load_monitoring() {
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       Speed & Load Monitoring Test                          ║");
  Serial.println("║  Testing servos 4 (SC) and 5 (STS) across all modes         ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  Serial.println("This test will run through 4 different scenarios:");
  Serial.println("  1. Position Mode - Watch speed/load during movement");
  Serial.println("  2. Load Testing - You resist movement with your fingers");
  Serial.println("  3. PWM Mode - Continuous rotation monitoring");
  Serial.println("  4. Wheel Mode - STS velocity control");
  Serial.println();
  Serial.println("Each test will wait for your input before proceeding.");

  wait_for_enter("Ready to begin?");

  auto sc_servo = SCServo(&servo_bus, 4);
  auto sts_servo = STSServo(&servo_bus, 5);

  // INITIALIZATION: Ensure clean state
  Serial.println("\n═══ INITIALIZATION ═══");
  Serial.println("Stopping any movement and resetting servos to position mode...");

  sc_servo.disable_torque();
  sts_servo.disable_torque();
  sc_servo.enable_position_mode();
  sts_servo.enable_position_mode();

  sc_servo.read_info();
  sts_servo.read_info();


  // Set STS servo to position mode
  Serial.println("Setting STS servo to position mode...");
  sts_servo.enable_position_mode();

  delay(500);

  // Enable torque
  sts_servo.enable_torque();
  Serial.println("Enabling torque...");
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  servo_bus.enable_torque(4);
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);
  servo_bus.enable_torque(5);

  delay(500);

  // Read servo info
  Serial.println("\nReading servo configurations...");
  if (!sc_servo.read_info()) {
    Serial.println("ERROR: Failed to read SC servo #4 info!");
    return;
  }
  if (!sts_servo.read_info()) {
    Serial.println("ERROR: Failed to read STS servo #5 info!");
    return;
  }

  Serial.printf("✓ SC Servo #4: min=%d, max=%d, range=%d\n",
                sc_servo.min_encoder_angle(),
                sc_servo.max_encoder_angle(),
                sc_servo.encoder_angle_range());
  Serial.printf("✓ STS Servo #5: min=%d, max=%d, range=%d\n\n",
                sts_servo.min_encoder_angle(),
                sts_servo.max_encoder_angle(),
                sts_servo.encoder_angle_range());

  // Helper lambda to print servo status
  auto print_servo_status = [](const char* label, Servo& servo) {
    auto pos = servo.read_encoder_angle();
    auto speed = servo.read_speed();
    auto load = servo.read_load();

    Serial.printf("%s | Pos: ", label);
    if (pos) Serial.printf("%4d", *pos);
    else Serial.print("FAIL");

    Serial.print(" | Speed: ");
    if (speed) Serial.printf("%5d", *speed);
    else Serial.print(" FAIL");

    Serial.print(" | Load: ");
    if (load) Serial.printf("%5d", *load);
    else Serial.print(" FAIL");

    Serial.println();
  };

  // ═══════════════════════════════════════════════════════════════
  // TEST 1: Position Mode Movement
  // ═══════════════════════════════════════════════════════════════
  Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║ TEST 1: Position Mode - Standard Movement                ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");

  Serial.println("\nIn this test, both servos will move to different positions.");
  Serial.println("Watch the speed values change during movement and return to 0 when stopped.");

  wait_for_enter("Ready to start TEST 1?");

  Serial.println("\nMoving both servos to center position (50%)...");
  sc_servo.move_to_percent(0.5f, 1000);
  sts_servo.move_to_percent(0.5f, 1000);

  Serial.println("\nMonitoring during movement (every 100ms):");
  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
  Serial.println("---------+------------+------+-------+------");

  for (int i = 0; i < 15; i++) {
    Serial.printf("%8d | ", millis());
    print_servo_status("SC #4 ", sc_servo);
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
    delay(100);
  }

  wait_for_enter("\nFirst movement complete. Notice speed returned to 0?");

  Serial.println("\nMoving to 75% position (slower, 1.5 seconds)...");
  sc_servo.move_to_percent(0.75f, 1500);
  sts_servo.move_to_percent(0.75f, 1500);

  Serial.println("\nMonitoring during movement:");
  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
  Serial.println("---------+------------+------+-------+------");

  for (int i = 0; i < 20; i++) {
    Serial.printf("%8d | ", millis());
    print_servo_status("SC #4 ", sc_servo);
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
    delay(100);
  }

  Serial.println("\n✓ TEST 1 Complete!");
  Serial.println("  - Speed shows actual velocity during movement");
  Serial.println("  - Speed returns to 0 when servo stops");
  Serial.println("  - Position updates smoothly to target");

  wait_for_enter();

  // ═══════════════════════════════════════════════════════════════
  // TEST 2: Load Testing with Reduced Torque Limit (STS only)
  // ═══════════════════════════════════════════════════════════════
  Serial.println("\n\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║ TEST 2: Load Testing - Reduced Torque Limit (STS only)   ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");

  Serial.println("\nIn this test, the STS servo torque will be reduced to make");
  Serial.println("it easy to resist with your fingers. Watch the LOAD value");
  Serial.println("increase as you apply resistance!");
  Serial.println();
  Serial.println("INSTRUCTIONS:");
  Serial.println("  1. Position your fingers near the STS servo horn");
  Serial.println("  2. When it starts moving, gently resist the movement");
  Serial.println("  3. Watch the 'Load' column in the output");
  Serial.println("  4. Try varying amounts of resistance");

  wait_for_enter("Ready for load testing?");

  uint16_t torque_limit = 80;
  Serial.printf("\nSetting STS servo torque limit to %d (out of 1023)...\n", torque_limit);
  if (sts_servo.set_torque_limit(80)) {
    auto limit = sts_servo.read_torque_limit();
    Serial.printf("✓ Torque limit set to: %d (easy to resist!)\n", limit);
  } else {
    Serial.println("✗ Failed to set torque limit!");
  }

  wait_for_enter("\nGet ready to resist the servo movement!");

  Serial.println("\n>>> STARTING MOVEMENT - RESIST WITH YOUR FINGERS! <<<");
  Serial.println("Moving STS servo to 25% position (3 seconds)...\n");

  uint32_t move_time = 10000;
  uint32_t move_start_time = millis();

  sts_servo.move_to_percent(0.25f, move_time);

  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load  <- Watch this!");
  Serial.println("---------+------------+------+-------+------");

  while (millis() - move_start_time < move_time) {
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
  }

  if (ask_yes_no("\nDid you see the Load value increase when you resisted?")) {
    Serial.println("✓ Great! Let's try it again in the other direction.");
  } else {
    Serial.println("  No problem - try applying more resistance this time.");
  }

  wait_for_enter();

  Serial.println("\n>>> STARTING REVERSE MOVEMENT - RESIST AGAIN! <<<");
  Serial.println("Moving back to 75% (3 seconds)...\n");

  sts_servo.move_to_percent(0.75f, 3000);

  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
  Serial.println("---------+------------+------+-------+------");

  for (int i = 0; i < 30; i++) {
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
    delay(100);
  }

  Serial.println("\n✓ TEST 2 Complete!");
  Serial.println("  - Load increases with resistance");
  Serial.println("  - Torque limit controls maximum force");
  Serial.println();

  // Restore torque limit
  Serial.println("Restoring full torque limit (1023)...");
  sts_servo.set_torque_limit(1023);

  wait_for_enter();

  // ═══════════════════════════════════════════════════════════════
  // TEST 3: PWM Mode (Both Servos)
  // ═══════════════════════════════════════════════════════════════
  Serial.println("\n\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║ TEST 3: PWM Mode - Open-Loop Speed Control               ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");

  Serial.println("\nPWM mode enables continuous rotation on both servos.");
  Serial.println("Watch how position continuously changes and speed is constant.");

  wait_for_enter("Ready to test PWM mode?");

  Serial.println("\nEnabling PWM mode on both servos...");
  if (!sc_servo.enable_pwm_mode()) {
    Serial.println("✗ Failed to enable PWM mode on SC servo!");
    return;
  }
  if (!sts_servo.enable_pwm_mode()) {
    Serial.println("✗ Failed to enable PWM mode on STS servo!");
    return;
  }
  Serial.println("✓ PWM mode enabled on both servos!");

  wait_for_enter("\nStarting clockwise rotation at speed 300...");

  sc_servo.set_pwm_speed(300);
  sts_servo.set_pwm_speed(300);

  Serial.println("\nMonitoring in PWM mode (CW):");
  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
  Serial.println("---------+------------+------+-------+------");

  for (int i = 0; i < 20; i++) {
    Serial.printf("%8d | ", millis());
    print_servo_status("SC #4 ", sc_servo);
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
    delay(100);
  }

  wait_for_enter("\nNotice position keeps changing? Now reversing direction...");

  sc_servo.set_pwm_speed(-300);
  sts_servo.set_pwm_speed(-300);

  Serial.println("\nMonitoring in PWM mode (CCW):");
  Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
  Serial.println("---------+------------+------+-------+------");

  for (int i = 0; i < 20; i++) {
    Serial.printf("%8d | ", millis());
    print_servo_status("SC #4 ", sc_servo);
    Serial.printf("%8d | ", millis());
    print_servo_status("STS #5", sts_servo);
    delay(100);
  }

  Serial.println("\nStopping PWM mode...");
  sc_servo.set_pwm_speed(0);
  sts_servo.set_pwm_speed(0);
  delay(500);

  Serial.println("\n✓ TEST 3 Complete!");
  Serial.println("  - PWM mode enables continuous rotation");
  Serial.println("  - Works on both SC and STS servos");
  Serial.println("  - Speed value reflects rotation velocity");

  wait_for_enter();

  // ═══════════════════════════════════════════════════════════════
  // TEST 4: Wheel Mode (STS only)
  // ═══════════════════════════════════════════════════════════════
  Serial.println("\n\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║ TEST 4: Wheel Mode - Velocity Control (STS only)         ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");

  Serial.println("\nWheel mode is unique to STS servos - it provides");
  Serial.println("velocity-controlled continuous rotation (different from PWM).");

  wait_for_enter("Ready to test wheel mode?");

  Serial.println("\nEnabling wheel mode on STS servo...");
  if (!sts_servo.enable_wheel_mode()) {
    Serial.println("✗ Failed to enable wheel mode!");
  } else {
    Serial.println("✓ Wheel mode enabled!");

    wait_for_enter("\nStarting rotation at velocity 400...");

    sts_servo.set_wheel_velocity(400);

    Serial.println("\nMonitoring in wheel mode (CW):");
    Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
    Serial.println("---------+------------+------+-------+------");

    for (int i = 0; i < 20; i++) {
      Serial.printf("%8d | ", millis());
      print_servo_status("STS #5", sts_servo);
      delay(100);
    }

    wait_for_enter("\nReversing wheel velocity...");

    sts_servo.set_wheel_velocity(-400);

    Serial.println("\nMonitoring in wheel mode (CCW):");
    Serial.println("Time(ms) | Servo      | Pos  | Speed | Load");
    Serial.println("---------+------------+------+-------+------");

    for (int i = 0; i < 20; i++) {
      Serial.printf("%8d | ", millis());
      print_servo_status("STS #5", sts_servo);
      delay(100);
    }

    Serial.println("\nStopping wheel...");
    sts_servo.set_wheel_velocity(0);
    delay(500);

    Serial.println("\n✓ TEST 4 Complete!");
    Serial.println("  - Wheel mode provides velocity control");
    Serial.println("  - STS-only feature (more precise than PWM)");
  }

  wait_for_enter();

  // ═══════════════════════════════════════════════════════════════
  // Restore Position Mode
  // ═══════════════════════════════════════════════════════════════
  Serial.println("\n\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              Restoring Servos to Position Mode            ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");

  Serial.println("\nBoth servos need to be returned to normal position mode.");

  wait_for_enter("Ready to restore position mode?");

  if (!sc_servo.enable_position_mode()) {
    Serial.println("✗ Failed to restore SC servo position mode!");
  } else {
    Serial.println("✓ SC servo position mode restored");
  }

  if (!sts_servo.enable_position_mode()) {
    Serial.println("✗ Failed to restore STS servo position mode!");
  } else {
    Serial.println("✓ STS servo position mode restored");
  }

  Serial.println("\nMoving both servos to center position...");
  sc_servo.move_to_percent(0.5f, 1000);
  sts_servo.move_to_percent(0.5f, 1000);
  delay(1500);

  Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              ALL TESTS COMPLETE!                          ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println("\n═══════════════════════════════════════════════════════════");
  Serial.println("                   TEST SUMMARY");
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  Serial.println("✓ TEST 1 - Position Mode Movement");
  Serial.println("    Speed monitoring during position changes");
  Serial.println();
  Serial.println("✓ TEST 2 - Load Testing");
  Serial.println("    Load increases with resistance (reduced torque limit)");
  Serial.println();
  Serial.println("✓ TEST 3 - PWM Mode");
  Serial.println("    Continuous rotation on both SC and STS servos");
  Serial.println();
  Serial.println("✓ TEST 4 - Wheel Mode");
  Serial.println("    Velocity-controlled rotation (STS only)");
  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  Serial.println("KEY FINDINGS:");
  Serial.println("  • read_speed() shows actual velocity");
  Serial.println("  • read_load() increases with resistance");
  Serial.println("  • PWM mode works on both servo types");
  Serial.println("  • Wheel mode is STS-specific");
  Serial.println("  • Position mode properly restored on both servos");
  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  Serial.println("You can now paste all test results back for analysis!");
}

void demonstrate_sync_write() {
  // Servo IDs to test
  std::vector<uint8_t> servo_ids = {4, 6};
  
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       Synchronized Position Writing Demo                    ║");
  Serial.print("║  Using sync_write_positions() with servo(s): ");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    Serial.printf("#%d", servo_ids[i]);
    if (i < servo_ids.size() - 1) Serial.print(", ");
  }
  Serial.println("          ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  // Assuming all servos are SC type
  servo_bus.set_servo_type(ServoBusApi::ServoType::SC);

  // Read servo info
  Serial.println("Reading servo configurations...");
  std::vector<ServoBusApi::ServoInfo> servo_infos;
  
  for (uint8_t id : servo_ids) {
    auto info = servo_bus.read_info(id);
    if (!info) {
      Serial.printf("ERROR: Failed to read servo #%d info!\n", id);
      Serial.println("Make sure all servos are connected.");
      return;
    }
    servo_infos.push_back(*info);
    Serial.printf("✓ Servo #%d: min=%d, max=%d\n", id, info->min_angle, info->max_angle);
  }

  // Enable torque
  Serial.println("\nEnabling torque...");
  for (uint8_t id : servo_ids) {
    servo_bus.enable_torque(id);
  }

  // Prepare vectors for sync write
  std::vector<uint16_t> positions(servo_ids.size());
  std::vector<uint16_t> times(servo_ids.size());
  std::vector<uint16_t> speeds(servo_ids.size());

  // Demo 1: Move all to center simultaneously
  Serial.println("\n=== Demo 1: All servos to center (sync) ===");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    positions[i] = (servo_infos[i].min_angle + servo_infos[i].max_angle) / 2;
    times[i] = 2000;
    speeds[i] = 500;
  }

  Serial.print("Moving ");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    Serial.printf("servo #%d to %d", servo_ids[i], positions[i]);
    if (i < servo_ids.size() - 1) Serial.print(", ");
  }
  Serial.println(" (2 sec)...");
  servo_bus.sync_write_positions(servo_ids, positions, times, speeds);
  delay(2500);

  // Demo 2: Mirror movement - servos move to opposite positions
  Serial.println("\n=== Demo 2: Mirror movement ===");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    positions[i] = (i % 2 == 0) ? servo_infos[i].min_angle : servo_infos[i].max_angle;
    times[i] = 3000;
    speeds[i] = 400;
  }

  for (size_t i = 0; i < servo_ids.size(); i++) {
    Serial.printf("Servo #%d → %d (%s)", servo_ids[i], positions[i], 
                  (i % 2 == 0) ? "min" : "max");
    if (i < servo_ids.size() - 1) Serial.print(", ");
  }
  Serial.println();
  servo_bus.sync_write_positions(servo_ids, positions, times, speeds);
  delay(3500);

  // Demo 3: Reverse mirror
  Serial.println("\n=== Demo 3: Reverse mirror ===");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    positions[i] = (i % 2 == 0) ? servo_infos[i].max_angle : servo_infos[i].min_angle;
    times[i] = 3000;
    speeds[i] = 400;
  }

  for (size_t i = 0; i < servo_ids.size(); i++) {
    Serial.printf("Servo #%d → %d (%s)", servo_ids[i], positions[i],
                  (i % 2 == 0) ? "max" : "min");
    if (i < servo_ids.size() - 1) Serial.print(", ");
  }
  Serial.println();
  servo_bus.sync_write_positions(servo_ids, positions, times, speeds);
  delay(3500);

  // Demo 4: Return to center
  Serial.println("\n=== Demo 4: All back to center ===");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    positions[i] = (servo_infos[i].min_angle + servo_infos[i].max_angle) / 2;
    times[i] = 2000;
    speeds[i] = 500;
  }

  servo_bus.sync_write_positions(servo_ids, positions, times, speeds);
  delay(2500);

  // Disable torque
  Serial.println("\nDisabling torque...");
  for (uint8_t id : servo_ids) {
    servo_bus.disable_torque(id);
  }

  Serial.println("\n✓ Synchronized Position Writing Demo Complete!");
  Serial.println("\nKey advantages of sync_write_positions():");
  Serial.println("  • Single bus transmission for all servos");
  Serial.println("  • Perfect timing synchronization");
  Serial.println("  • Reduced latency compared to sequential writes");
}

void demonstrate_sync_read() {
  // Servo IDs to test
  std::vector<uint8_t> servo_ids = {5};
  
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       Synchronized Position Reading Demo                    ║");
  Serial.print("║  Using sync_read_positions() with servo(s): ");
  for (size_t i = 0; i < servo_ids.size(); i++) {
    Serial.printf("#%d", servo_ids[i]);
    if (i < servo_ids.size() - 1) Serial.print(", ");
  }
  Serial.println("             ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  Serial.println("STS servos support SYNC_READ (0x82) for efficient bulk reads.\n");

  // Set servo type
  servo_bus.set_servo_type(ServoBusApi::ServoType::STS);

  // Read servo info and prepare positions
  Serial.println("Reading servo configuration...");
  std::vector<uint16_t> positions;
  std::vector<uint16_t> times;
  std::vector<uint16_t> speeds;
  
  for (uint8_t id : servo_ids) {
    auto info = servo_bus.read_info(id);
    if (!info) {
      Serial.printf("ERROR: Failed to read servo #%d info!\n", id);
      return;
    }
    Serial.printf("✓ Servo #%d: min=%d, max=%d\n", id, info->min_angle, info->max_angle);
    
    uint16_t center_pos = (info->min_angle + info->max_angle) / 2;
    positions.push_back(center_pos);
    times.push_back(1000);
    speeds.push_back(500);
  }

  // Enable torque and move to center positions
  Serial.println("\nEnabling torque and moving to center positions...");
  for (uint8_t id : servo_ids) {
    servo_bus.enable_torque(id);
  }

  servo_bus.sync_write_positions(servo_ids, positions, times, speeds);
  delay(1500);

  Serial.println("Servos moved to center positions");

  // Compare individual read vs sync_read
  Serial.println("\n=== Comparison: Individual Read vs Sync Read ===\n");
  
  // Method 1: Individual reads
  Serial.println("Method 1: Individual read_position()");
  std::vector<std::optional<int>> individual_positions;
  for (uint8_t id : servo_ids) {
    auto pos = servo_bus.read_position(id);
    individual_positions.push_back(pos);
    if (pos) {
      Serial.printf("  Servo #%d: %d\n", id, *pos);
    } else {
      Serial.printf("  Servo #%d: FAILED\n", id);
    }
  }
  
  // Method 2: Sync read
  Serial.println("\nMethod 2: sync_read_positions()");
  auto sync_results = servo_bus.sync_read_positions(servo_ids);
  
  for (size_t i = 0; i < servo_ids.size(); i++) {
    if (i < sync_results.size() && sync_results[i]) {
      Serial.printf("  Servo #%d: %d\n", servo_ids[i], *sync_results[i]);
      
      // Compare with individual read
      if (i < individual_positions.size() && individual_positions[i] && sync_results[i]) {
        int diff = abs(*individual_positions[i] - *sync_results[i]);
        if (diff <= 2) {
          Serial.printf("    ✓ Matches individual read (diff: %d)\n", diff);
        } else {
          Serial.printf("    ⚠ Mismatch with individual read (diff: %d)\n", diff);
        }
      }
    } else {
      Serial.printf("  Servo #%d: FAILED\n", servo_ids[i]);
    }
  }
  
  Serial.println("\n=== Testing Multiple Reads ===");
  Serial.println("Reading positions 5 times with sync_read...\n");
  
  for (int i = 0; i < 5; i++) {
    auto results = servo_bus.sync_read_positions(servo_ids);
    Serial.printf("Read %d: ", i + 1);
    for (size_t j = 0; j < servo_ids.size(); j++) {
      if (j < results.size() && results[j]) {
        Serial.printf("#%d=%d", servo_ids[j], *results[j]);
      } else {
        Serial.printf("#%d=FAIL", servo_ids[j]);
      }
      if (j < servo_ids.size() - 1) Serial.print(", ");
    }
    Serial.println();
    delay(100);
  }

  // Disable torque
  Serial.println("\nDisabling torque...");
  for (uint8_t id : servo_ids) {
    servo_bus.disable_torque(id);
  }
  
  Serial.println("\n✓ Sync Read Demo Complete!");
  Serial.println("\nKey advantages of sync_read_positions():");
  Serial.println("  • Single bus transaction reads multiple servos");
  Serial.println("  • Reduced latency compared to sequential reads");
  Serial.println("  • Synchronized snapshot of all servo positions");
  Serial.println("  • Works on STS servos (SC servos typically don't support it)");
}

void setup() {
  Serial.begin(1000000);
  delay(2000);

  Serial1.begin(1000000, SERIAL_8N1, pin_servo_rx, pin_servo_tx);

// Configure TX as open-drain (only pulls LOW, floats HIGH)
//gpio_set_pull_mode((gpio_num_t)pin_servo_tx, GPIO_PULLUP_ONLY);  // Enable pull-up
// gpio_set_drive_capability((gpio_num_t)pin_servo_tx, GPIO_DRIVE_CAP_2);  // Medium drive (10mA)

// // Make TX pin open-drain
// PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_servo_tx], PIN_FUNC_GPIO);
// gpio_set_direction((gpio_num_t)pin_servo_tx, GPIO_MODE_OUTPUT_OD);  // Open-drain mode
// gpio_matrix_out(pin_servo_tx, U1TXD_OUT_IDX, false, false);  // Reconnect UART TX

  servo_bus.set_serial(&Serial1);

  // Wait a bit after serial initialization
  delay(500);

  // === ID CONFIGURATION TOOLS ===
  // Uncomment these to diagnose and fix servo ID issues


  // demonstrate_scan_ids();
  // demonstrate_voltage();
  // demonstrate_temperature();
  // demonstrate_infer_servo_type();


  // servo_bus.set_servo_type(ServoBusApi::ServoType::SC);
  // servo_bus.set_servo_id_permanent(1, 6);
  // scan_ids(); 

  // 3. After setting IDs, power cycle each servo and scan again to verify



  // === ACTIVE TEST ===
  // diagnose_eprom_registers();           // Check what's actually in the servo EPROM
  // restore_sts_eprom();                  // Fix the corrupted STS servo EPROM
  // test_speed_and_load_monitoring();     // Comprehensive speed & load test

  // Other demos available (currently not running):
  // emergency_servo_reset();              // Emergency recovery for stuck servos
  // demonstrate_pwm_mode();               // PWM mode demo - works on all servos
  // demonstrate_coordinated_moving();     // Original version with manual type switching
  // demonstrate_coordinated_moving_2();   // Clean version using servo objects
  // demonstrate_sts_features();           // Full STS features demo
  // demonstrate_wheel_mode();             // STS-only wheel mode demo
  demonstrate_sync_write();             // Sync write demo with servos 4 & 6
  // demonstrate_sync_read();              // Sync read experiment with servos 4 & 6
}

void loop() {}