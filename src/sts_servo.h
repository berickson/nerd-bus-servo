// STS series servo (little-endian)
class STSServo : public Servo {
public:
  using Servo::Servo;
  ServoBusApi::ServoType type() const override { return ServoBusApi::ServoType::STS; }
  
  // Velocity-based wheel mode - unique to STS servos!
  // This uses MODE register to enable true continuous rotation with velocity control
  // (Different from PWM mode which is available on all servos)
  bool enable_wheel_mode() {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());
    return bus_->enable_wheel_mode(id_);
  }
  
  bool set_wheel_velocity(int16_t speed) {
    bus_->set_servo_type(type());
    return bus_->set_wheel_velocity(id_, speed);
  }

  // Hardware acceleration control - unique to STS servos!
  // ACC parameter (0-255) controls velocity ramping for smooth motion
  // ACC=0: No acceleration (immediate speed changes)
  // ACC=50: Moderate smoothing
  // ACC=200+: High smoothing (gradual ramp-up/ramp-down)
  bool move_to_encoder_angle_with_accel(uint16_t encoder_angle, uint16_t speed, uint8_t acc) {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());
    return bus_->write_position_sts_with_accel(id_, encoder_angle, speed, acc);
  }

  bool move_to_percent_with_accel(float percent, uint16_t speed, uint8_t acc) {
    if (!info_loaded_) return false;
    percent = constrain(percent, 0.0f, 1.0f);
    uint16_t encoder_angle = min_encoder_angle_ + (uint16_t)(percent * encoder_angle_range());
    return move_to_encoder_angle_with_accel(encoder_angle, speed, acc);
  }

  // Torque limit control - unique to STS servos!
  // Limits maximum torque output (0-1023, default 1023 = 100%)
  // Useful for preventing damage or controlling force
  bool set_torque_limit(uint16_t limit) {
    bus_->set_servo_type(type());
    return bus_->set_torque_limit(id_, limit);
  }

  uint16_t read_torque_limit() {
    bus_->set_servo_type(type());
    return bus_->read_torque_limit(id_);
  }
};
