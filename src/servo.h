#pragma once

#include "servo_bus_api.h"

// Base servo class
class Servo {
protected:
  ServoBusApi* bus_;
  uint8_t id_;
  uint16_t min_encoder_angle_ = 0;
  uint16_t max_encoder_angle_ = 4095;
  bool info_loaded_ = false;

public:
  Servo(ServoBusApi* bus, uint8_t id) : bus_(bus), id_(id) {}
  virtual ~Servo() = default;
  
  uint8_t id() const { return id_; }
  uint16_t min_encoder_angle() const { return min_encoder_angle_; }
  uint16_t max_encoder_angle() const { return max_encoder_angle_; }
  uint16_t encoder_angle_range() const { return max_encoder_angle_ - min_encoder_angle_; }
  bool info_loaded() const { return info_loaded_; }
  
  virtual ServoBusApi::ServoType type() const = 0;
  
  std::optional<ServoBusApi::ServoInfo> read_info() {
    bus_->set_servo_type(type());
    auto info = bus_->read_info(id_);
    if (info) {
      min_encoder_angle_ = info->min_angle;
      max_encoder_angle_ = info->max_angle;
      info_loaded_ = true;
    }
    return info;
  }
  
  std::optional<int> read_encoder_angle() {
    bus_->set_servo_type(type());
    return bus_->read_position(id_);
  }

  std::optional<int16_t> read_speed() {
    bus_->set_servo_type(type());
    return bus_->read_speed(id_);
  }

  std::optional<int16_t> read_load() {
    bus_->set_servo_type(type());
    return bus_->read_load(id_);
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
