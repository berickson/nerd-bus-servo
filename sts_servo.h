// STS series servo (little-endian)
class STSServo : public Servo {
public:
  using Servo::Servo;
  ServoBusApi::ServoType type() const override { return ServoBusApi::ServoType::STS; }
  
  // STS motor speed uses wheel velocity (true velocity control)
  // rather than PWM open-loop mode
  bool set_motor_speed(int16_t speed) override {
    bus_->set_servo_type(type());
    return bus_->set_wheel_velocity(id_, speed);
  }
  
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

  struct SafetyConfig {
    uint8_t max_temp;
    uint8_t max_voltage;        // raw, ×0.1V
    uint8_t min_voltage;        // raw, ×0.1V
    uint16_t max_torque;        // 0-1000
    uint16_t protection_current; // raw, ×6.5mA
    uint8_t protective_torque;  // 0-100, 1% units
    uint8_t protection_time;    // ×10ms
    uint8_t overload_torque;    // 0-100, 1% units
    uint8_t overcurrent_prot_time; // ×10ms
    uint8_t unload_conditions;  // bitmask: which protections cut torque
    uint8_t led_alarm_conditions; // bitmask: which protections blink LED
  };

  std::optional<SafetyConfig> read_safety_config() {
    bus_->set_servo_type(type());
    SafetyConfig cfg;
    auto v = bus_->read_byte(id_, ServoBusApi::Register::max_temp_limit);
    if (!v) return std::nullopt;
    cfg.max_temp = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::max_voltage_limit);
    if (!v) return std::nullopt;
    cfg.max_voltage = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::min_voltage_limit);
    if (!v) return std::nullopt;
    cfg.min_voltage = *v;
    auto w = bus_->read_word(id_, ServoBusApi::Register::max_torque_limit_l);
    if (!w) return std::nullopt;
    cfg.max_torque = *w;
    w = bus_->read_word(id_, ServoBusApi::Register::protection_current_l);
    if (!w) return std::nullopt;
    cfg.protection_current = *w;
    v = bus_->read_byte(id_, ServoBusApi::Register::protective_torque);
    if (!v) return std::nullopt;
    cfg.protective_torque = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::protection_time);
    if (!v) return std::nullopt;
    cfg.protection_time = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::overload_torque);
    if (!v) return std::nullopt;
    cfg.overload_torque = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::overcurrent_prot_time);
    if (!v) return std::nullopt;
    cfg.overcurrent_prot_time = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::unload_conditions);
    if (!v) return std::nullopt;
    cfg.unload_conditions = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::led_alarm_conditions);
    if (!v) return std::nullopt;
    cfg.led_alarm_conditions = *v;
    return cfg;
  }

  bool write_safety_config(const SafetyConfig& cfg) {
    bus_->set_servo_type(type());
    if (!bus_->unlock_eeprom(id_)) return false;
    delay(10);
    bool ok = true;
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::max_temp_limit, cfg.max_temp);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::max_voltage_limit, cfg.max_voltage);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::min_voltage_limit, cfg.min_voltage);
    ok = ok && bus_->write_word(id_, ServoBusApi::Register::max_torque_limit_l, cfg.max_torque);
    ok = ok && bus_->write_word(id_, ServoBusApi::Register::protection_current_l, cfg.protection_current);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::protective_torque, cfg.protective_torque);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::protection_time, cfg.protection_time);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::overload_torque, cfg.overload_torque);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::overcurrent_prot_time, cfg.overcurrent_prot_time);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::unload_conditions, cfg.unload_conditions);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::led_alarm_conditions, cfg.led_alarm_conditions);
    delay(10);
    bus_->lock_eeprom(id_);
    return ok;
  }

  struct TuningConfig {
    uint8_t p_coefficient;
    uint8_t d_coefficient;
    uint8_t i_coefficient;
    uint16_t min_starting_force;  // 0-1000, 0.1% units
    uint8_t cw_dead;              // CW dead zone in steps (0-32)
    uint8_t ccw_dead;             // CCW dead zone in steps (0-32)
  };

  std::optional<TuningConfig> read_tuning_config() {
    bus_->set_servo_type(type());
    TuningConfig cfg;
    auto v = bus_->read_byte(id_, ServoBusApi::Register::p_coefficient);
    if (!v) return std::nullopt;
    cfg.p_coefficient = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::d_coefficient);
    if (!v) return std::nullopt;
    cfg.d_coefficient = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::i_coefficient);
    if (!v) return std::nullopt;
    cfg.i_coefficient = *v;
    auto w = bus_->read_word(id_, ServoBusApi::Register::min_starting_force_l);
    if (!w) return std::nullopt;
    cfg.min_starting_force = *w;
    v = bus_->read_byte(id_, ServoBusApi::Register::cw_dead);
    if (!v) return std::nullopt;
    cfg.cw_dead = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::ccw_dead);
    if (!v) return std::nullopt;
    cfg.ccw_dead = *v;
    return cfg;
  }

  bool write_tuning_config(const TuningConfig& cfg) {
    bus_->set_servo_type(type());
    if (!bus_->unlock_eeprom(id_)) return false;
    delay(10);
    bool ok = true;
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::p_coefficient, cfg.p_coefficient);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::d_coefficient, cfg.d_coefficient);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::i_coefficient, cfg.i_coefficient);
    ok = ok && bus_->write_word(id_, ServoBusApi::Register::min_starting_force_l, cfg.min_starting_force);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::cw_dead, cfg.cw_dead);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::ccw_dead, cfg.ccw_dead);
    delay(10);
    bus_->lock_eeprom(id_);
    return ok;
  }
};
