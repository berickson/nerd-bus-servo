#pragma once

#include "servo_bus_api.h"

// Known servo model versions from manufacturer specs (registers 3-4)
// Add entries here as new models are discovered from real hardware
struct ServoModelEntry {
  uint8_t version_major;  // register 3
  uint8_t version_minor;  // register 4
  ServoBusApi::ServoType type;
  const char* name;
};

static constexpr ServoModelEntry known_models[] = {
  // SC series (big-endian, 10-bit)
  {5, 4,  ServoBusApi::ServoType::SC,  "SCS"},      // confirmed on real hardware
  {5, 15, ServoBusApi::ServoType::SC,  "SCS"},      // from SCS_Series_Memory_Table_Analysis.csv
  // STS series (little-endian, 12-bit)
  {9, 3,  ServoBusApi::ServoType::STS, "ST3215"},   // from ST3215 memory register map-EN.csv
};

static std::optional<const ServoModelEntry*> lookup_model(uint8_t major, uint8_t minor) {
  for (const auto& entry : known_models) {
    if (entry.version_major == major && entry.version_minor == minor) {
      return &entry;
    }
  }
  return std::nullopt;
}

// Base servo class
class Servo {
protected:
  ServoBusApi* bus_;
  uint8_t id_;
  uint16_t min_encoder_angle_ = 0;
  uint16_t max_encoder_angle_ = 4095;
  bool info_loaded_ = false;
  bool current_supported_ = false;

public:
  Servo(ServoBusApi* bus, uint8_t id) : bus_(bus), id_(id) {}
  virtual ~Servo() = default;
  
  uint8_t id() const { return id_; }
  uint16_t min_encoder_angle() const { return min_encoder_angle_; }
  uint16_t max_encoder_angle() const { return max_encoder_angle_; }
  uint16_t encoder_angle_range() const { return max_encoder_angle_ - min_encoder_angle_; }
  uint16_t full_range() const { return (type() == ServoBusApi::ServoType::STS) ? 4095 : 1023; }
  bool info_loaded() const { return info_loaded_; }
  bool current_supported() const { return current_supported_; }
  
  // Detect servo type using version registers, falling back to angle limit heuristic
  static std::optional<ServoBusApi::ServoType> infer_servo_type(ServoBusApi* bus, uint8_t id) {
    // Step 1: Read version registers (3-4). These are individual bytes,
    // so byte order is irrelevant — works regardless of servo type setting.
    bus->set_servo_type(ServoBusApi::ServoType::STS);
    auto ver_raw = bus->read_register(id, ServoBusApi::Register::version_l, 2);
    if (ver_raw) {
      uint8_t version_major = ver_raw[0];
      uint8_t version_minor = ver_raw[1];

      auto match = lookup_model(version_major, version_minor);
      if (match) {
        Serial.printf("Servo %d: version %d.%d → %s (known model)\n",
                       id, version_major, version_minor, (*match)->name);
        return (*match)->type;
      }
      Serial.printf("Servo %d: version %d.%d (unknown model, using angle heuristic)\n",
                     id, version_major, version_minor);
    }

    // Step 2: Angle limit heuristic fallback for unknown models
    // Read raw bytes once and check both byte-order interpretations
    auto raw_max = bus->read_register(id, ServoBusApi::Register::max_angle_limit_l, 2);
    if (!raw_max) return std::nullopt;

    uint8_t b0 = raw_max[0];
    uint8_t b1 = raw_max[1];
    uint16_t as_sts = b0 | (b1 << 8);         // little-endian interpretation
    uint16_t as_sc  = (b0 << 8) | b1;         // big-endian interpretation
    bool valid_sts = (as_sts >= 1024 && as_sts <= 4095);
    bool valid_sc  = (as_sc <= 1023);

    Serial.printf("Servo %d: angle raw [0x%02X,0x%02X] as_sts=%d as_sc=%d\n",
                   id, b0, b1, as_sts, as_sc);

    if (valid_sts && !valid_sc) return ServoBusApi::ServoType::STS;
    if (valid_sc && !valid_sts) return ServoBusApi::ServoType::SC;

    // Both zero → likely wheel/motor mode, check MODE register
    if (as_sts == 0) {
      auto mode = bus->read_mode(id);
      if (mode.has_value() && mode.value() != 0) {
        return ServoBusApi::ServoType::STS;
      }
    }

    // Ambiguous (both interpretations valid) or neither valid
    Serial.printf("Servo %d: type ambiguous (sts=%d sc=%d)\n", id, as_sts, as_sc);
    return std::nullopt;
  }
  
  virtual ServoBusApi::ServoType type() const = 0;
  
  std::optional<ServoBusApi::ServoInfo> read_info() {
    bus_->set_servo_type(type());
    auto info = bus_->read_info(id_);
    if (info) {
      min_encoder_angle_ = info->min_angle;
      max_encoder_angle_ = info->max_angle;
      info_loaded_ = true;
    }
    // SC servos do not have current sensing hardware — register 69-70
    // exists but always returns 0.  Only STS servos report real current.
    current_supported_ = (type() == ServoBusApi::ServoType::STS);
    return info;
  }
  
  std::optional<int> read_encoder_angle() {
    bus_->set_servo_type(type());
    return bus_->read_position(id_);
  }

  std::optional<int> read_goal_position() {
    bus_->set_servo_type(type());
    return bus_->read_goal_position(id_);
  }

  std::optional<int16_t> read_speed() {
    bus_->set_servo_type(type());
    return bus_->read_speed(id_);
  }

  std::optional<int16_t> read_load() {
    bus_->set_servo_type(type());
    return bus_->read_load(id_);
  }

  std::optional<float> read_voltage() {
    bus_->set_servo_type(type());
    auto voltage = bus_->read_voltage(id_);
    if (voltage) {
      return float(*voltage) / 10.0;
    } else {
      return std::nullopt;
    }
  }

    std::optional<float> read_temperature() {
    bus_->set_servo_type(type());
    auto temperature = bus_->read_temperature(id_);
    if (temperature) {
      return float(*temperature);
    } else {
      return std::nullopt;
    }
  }

  std::optional<int16_t> read_current() {
    if (!current_supported_) return std::nullopt;
    bus_->set_servo_type(type());
    return bus_->read_current(id_);
  }

  std::optional<uint8_t> read_alarm_status() {
    bus_->set_servo_type(type());
    return bus_->read_byte(id_, ServoBusApi::Register::servo_status);
  }

  // Read operating mode (0=position, 3=motor; SC infers from angle limits)
  std::optional<uint8_t> read_mode() {
    bus_->set_servo_type(type());
    return bus_->read_mode(id_);
  }

  // Read angle limits from EEPROM
  std::optional<ServoBusApi::AngleLimits> read_angle_limits() {
    bus_->set_servo_type(type());
    return bus_->read_angle_limits(id_);
  }

  // Write angle limits to EEPROM (with unlock/lock cycle)
  bool write_angle_limits(uint16_t min_angle, uint16_t max_angle) {
    bus_->set_servo_type(type());
    if (!bus_->unlock_eeprom(id_)) return false;
    delay(10);
    bool ok = bus_->write_angle_limits(id_, min_angle, max_angle);
    delay(10);
    bus_->lock_eeprom(id_);
    if (ok) {
      min_encoder_angle_ = min_angle;
      max_encoder_angle_ = max_angle;
    }
    return ok;
  }

  // Change servo ID permanently (with EEPROM unlock/lock)
  bool set_id(uint8_t new_id) {
    bus_->set_servo_type(type());
    bool ok = bus_->set_servo_id_permanent(id_, new_id);
    if (ok) id_ = new_id;
    return ok;
  }

  bool unlock_eeprom() {
    bus_->set_servo_type(type());
    return bus_->unlock_eeprom(id_);
  }

  bool lock_eeprom() {
    bus_->set_servo_type(type());
    return bus_->lock_eeprom(id_);
  }

  // Enable motor (continuous rotation) mode with EEPROM lock/unlock
  bool enable_motor_mode() {
    bus_->set_servo_type(type());
    if (!bus_->unlock_eeprom(id_)) return false;
    delay(10);
    bool ok = bus_->enable_motor_mode(id_);
    delay(10);
    bus_->lock_eeprom(id_);
    return ok;
  }

  // Restore position mode with EEPROM lock/unlock
  // If angle limits haven't been read, uses type-appropriate defaults:
  //   SC:  min=20, max=1003
  //   STS: min=0,  max=4095
  bool restore_position_mode() {
    bus_->set_servo_type(type());
    uint16_t min_angle = min_encoder_angle_;
    uint16_t max_angle = max_encoder_angle_;
    if (!info_loaded_) {
      if (type() == ServoBusApi::ServoType::STS) {
        min_angle = 0;
        max_angle = 4095;
      } else {
        min_angle = 20;
        max_angle = 1003;
      }
    }
    if (!bus_->unlock_eeprom(id_)) return false;
    delay(10);
    bool ok = bus_->enable_position_mode(id_, min_angle, max_angle);
    delay(10);
    bus_->lock_eeprom(id_);
    return ok;
  }

  // Set motor speed in motor/wheel mode
  // Positive = CW, negative = CCW
  // SC servos: uses PWM mode speed
  // STS servos: uses wheel velocity (override in STSServo)
  virtual bool set_motor_speed(int16_t speed) {
    bus_->set_servo_type(type());
    return set_pwm_speed(speed);
  }



  bool enable_torque() {
    bus_->set_servo_type(type());
    return bus_->enable_torque(id_);
  }

  bool disable_torque() {
    bus_->set_servo_type(type());
    return bus_->disable_torque(id_);
  }

  std::optional<bool> is_torque_enabled() {
    bus_->set_servo_type(type());
    return bus_->read_torque_enabled(id_);
  }

  bool move_to_encoder_angle(uint16_t encoder_angle, uint32_t duration_ms) {
    if (!info_loaded_) return false;
    if (duration_ms == 0) return false;
    bus_->set_servo_type(type());
    
    auto current = read_encoder_angle();
    if (!current) return false;
    
    uint16_t distance = abs((int)encoder_angle - *current);
    uint16_t speed = (distance * 1000) / duration_ms;
    return bus_->write_position(id_, encoder_angle, duration_ms, speed);
  }
  
  bool move_to_percent(float percent, uint32_t duration_ms) {
    if (!info_loaded_) return false;
    percent = constrain(percent, 0.0f, 1.0f);
    uint16_t encoder_angle = min_encoder_angle_ + (uint16_t)(percent * encoder_angle_range());
    return move_to_encoder_angle(encoder_angle, duration_ms);
  }
  
  // PWM/Motor mode - available on all servo types
  // SC servos: Set angle limits to 0 to enable PWM mode
  // STS servos: Set MODE register to 2 (PWM open-loop mode)
  // Then writing PWM values to GOAL_TIME register (not GOAL_SPEED!)
  bool enable_pwm_mode() {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());

    if (type() == ServoBusApi::ServoType::STS) {
      // STS servos: Set MODE register to 2 for PWM open-loop mode
      uint8_t mode_params[2];
      mode_params[0] = ServoBusApi::to_byte(ServoBusApi::Register::mode);
      mode_params[1] = 2;  // Mode 2 = PWM open-loop speed regulation

      if(!bus_->send_command(id_, ServoBusApi::Instruction::write, mode_params, 2)) {
        return false;
      }

      uint8_t response[ServoBusApi::MIN_PACKET_SIZE];
      if(!bus_->read_response(response, ServoBusApi::MIN_PACKET_SIZE)) {
        return false;
      }
    } else {
      // SC servos: Set min and max angle limits to 0 to enable PWM mode
      uint8_t params[5];
      params[0] = ServoBusApi::to_byte(ServoBusApi::Register::min_angle_limit_l);
      params[1] = 0;  // min_angle_l
      params[2] = 0;  // min_angle_h
      params[3] = 0;  // max_angle_l
      params[4] = 0;  // max_angle_h

      if(!bus_->send_command(id_, ServoBusApi::Instruction::write, params, 5)) {
        return false;
      }

      uint8_t response[ServoBusApi::MIN_PACKET_SIZE];
      if(!bus_->read_response(response, ServoBusApi::MIN_PACKET_SIZE)) {
        return false;
      }
    }

    return true;
  }
  
  bool set_pwm_speed(int16_t speed) {
    bus_->set_servo_type(type());

    // PWM format: use bit 10 for direction
    // Positive = CW, Negative = CCW
    uint16_t pwm_value;
    if (speed < 0) {
      pwm_value = (-speed) | (1 << 10);  // Set bit 10 for reverse
    } else {
      pwm_value = speed;
    }

    // Write to GOAL_TIME register (not GOAL_SPEED!)
    uint8_t parameters[3];
    parameters[0] = ServoBusApi::to_byte(ServoBusApi::Register::goal_time_l);

    // Pack using configured byte order
    uint8_t temp[2];
    bus_->set_servo_type(type());
    if (type() == ServoBusApi::ServoType::STS) {
      temp[0] = pwm_value & 0xFF;
      temp[1] = (pwm_value >> 8) & 0xFF;
    } else {
      temp[0] = (pwm_value >> 8) & 0xFF;
      temp[1] = pwm_value & 0xFF;
    }
    parameters[1] = temp[0];
    parameters[2] = temp[1];

    if(!bus_->send_command(id_, ServoBusApi::Instruction::write, parameters, 3)) {
      return false;
    }

    uint8_t response[ServoBusApi::MIN_PACKET_SIZE];
    return bus_->read_response(response, ServoBusApi::MIN_PACKET_SIZE);
  }

  // Restore position mode after PWM or wheel mode
  // SC servos: Restores angle limits (disabled by PWM mode)
  // STS servos: Sets MODE register back to 0 (position mode)
  bool enable_position_mode() {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());
    bool result = bus_->enable_position_mode(id_, min_encoder_angle_, max_encoder_angle_);
    if (result) {
      // Re-read info to ensure cached values are correct
      read_info();
    }
    return result;
  }
};
