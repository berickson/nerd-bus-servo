#include "servo.h"

// SC series servo (big-endian)
class SCServo : public Servo {
public:
  using Servo::Servo;
  ServoBusApi::ServoType type() const override { return ServoBusApi::ServoType::SC; }

  struct SafetyConfig {
    uint8_t max_temp;
    uint8_t max_voltage;        // raw, ×0.1V
    uint8_t min_voltage;        // raw, ×0.1V
    uint16_t max_torque;        // 0-1000
    uint8_t protective_torque;  // 0-100, 1% units
    uint8_t protection_time;    // ×40ms
    uint8_t overload_torque;    // 0-100, 1% units
    uint8_t unload_conditions;  // bitmask: bit0=Volt, bit2=Temp, bit5=Overload
    uint8_t led_alarm_conditions;
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
    v = bus_->read_byte(id_, ServoBusApi::Register::sc_protective_torque);
    if (!v) return std::nullopt;
    cfg.protective_torque = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::sc_protection_time);
    if (!v) return std::nullopt;
    cfg.protection_time = *v;
    v = bus_->read_byte(id_, ServoBusApi::Register::sc_overload_torque);
    if (!v) return std::nullopt;
    cfg.overload_torque = *v;
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
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::sc_protective_torque, cfg.protective_torque);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::sc_protection_time, cfg.protection_time);
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::sc_overload_torque, cfg.overload_torque);
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
    uint8_t hysteresis;           // Positioning deadband in steps (0-32)
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
    v = bus_->read_byte(id_, ServoBusApi::Register::sc_hysteresis);
    if (!v) return std::nullopt;
    cfg.hysteresis = *v;
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
    ok = ok && bus_->write_byte(id_, ServoBusApi::Register::sc_hysteresis, cfg.hysteresis);
    delay(10);
    bus_->lock_eeprom(id_);
    return ok;
  }
};
