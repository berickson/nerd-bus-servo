#include <Arduino.h>

#include <optional>
#include <vector>

#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define pin_servo_tx 8
#define pin_servo_rx 18


class SCServoBus {
public:
  // Protocol byte constants
  enum class Protocol : uint8_t {
    header_byte_1 = 0xFF,
    header_byte_2 = 0xFF,
    broadcast_id = 0xFE
  };
  
  // Servo type for byte order
  enum class ServoType {
    SC,   // SCSCL series - big-endian (HIGH, LOW)
    STS   // SMS_STS series - little-endian (LOW, HIGH)
  };
  
  // Protocol size constants
  static constexpr int HEADER_SIZE = 2;
  static constexpr int MIN_PACKET_SIZE = 6;  // header(2) + id(1) + length(1) + instruction(1) + checksum(1)
  static constexpr int INSTRUCTION_OVERHEAD = 2;  // instruction byte + params (length field = instruction + params)
  
  // Packet field offsets
  enum class PacketOffset : int {
    header_1 = 0,
    header_2 = 1,
    id = 2,
    length = 3,
    instruction = 4,
    parameters = 5
  };

  // Servo memory register addresses (from SCSCL.h)
  enum class Register : uint8_t {
    // EPROM (read-only)
    version_l = 3,
    version_h = 4,
    
    // EPROM (read/write)
    id = 5,
    baud_rate = 6,
    min_angle_limit_l = 9,
    min_angle_limit_h = 10,
    max_angle_limit_l = 11,
    max_angle_limit_h = 12,
    cw_dead = 26,
    ccw_dead = 27,
    ofs_l = 31,      // Offset calibration low byte (STS only)
    ofs_h = 32,      // Offset calibration high byte (STS only)
    mode = 33,       // Servo mode: 0=position, 1=wheel (continuous rotation) (STS only)
    
    // SRAM (read/write)
    torque_enable = 40,
    acc = 41,  // Acceleration control (0-255) - STS servos only
    goal_position_l = 42,
    goal_position_h = 43,
    goal_time_l = 44,
    goal_time_h = 45,
    goal_speed_l = 46,
    goal_speed_h = 47,
    lock_sc = 48,          // LOCK register for SC servos (SCSCL)
    torque_limit_l = 48,   // Torque limit low byte (STS only) - CONFLICTS with SC LOCK!
    torque_limit_h = 49,   // Torque limit high byte (STS only)
    lock_sts = 55,         // LOCK register for STS servos (SMS_STS)
    
    // SRAM (read-only)
    present_position_l = 56,
    present_position_h = 57,
    present_speed_l = 58,
    present_speed_h = 59,
    present_load_l = 60,
    present_load_h = 61,
    present_voltage = 62,
    present_temperature = 63,
    moving = 66,
    present_current_l = 69,
    present_current_h = 70
  };

  // Protocol instruction codes (from INST.h)
  enum class Instruction : uint8_t {
    ping = 0x01,
    read = 0x02,
    write = 0x03,
    reg_write = 0x04,
    reg_action = 0x05,
    sync_read = 0x82,
    sync_write = 0x83
  };

  enum class ServoError {
    none = 0,
    timeout,
    invalid_header,
    checksum_mismatch,
    invalid_response,
    invalid_parameter,
    no_ack
  };

  // Helper functions to convert enums to underlying types
  static constexpr uint8_t to_byte(Protocol p) { return static_cast<uint8_t>(p); }
  static constexpr uint8_t to_byte(Register r) { return static_cast<uint8_t>(r); }
  static constexpr uint8_t to_byte(Instruction i) { return static_cast<uint8_t>(i); }
  static constexpr int to_index(PacketOffset offset) { return static_cast<int>(offset); }

  private:
  ServoError last_error_ = ServoError::none;
  HardwareSerial* bus_serial_ = nullptr;
  uint32_t timeout_ms_ = 10;
  static const uint32_t max_servo_count = 10; // maximum servo count supported at once
  ServoType servo_type_ = ServoType::STS;  // Default to STS
  
  // Pack 16-bit value into big-endian byte array (HIGH byte first) - for SC servos
  void pack_uint16_be(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;  // HIGH byte
    buffer[1] = value & 0xFF;         // LOW byte
  }
  
  // Unpack 16-bit value from big-endian byte array (HIGH byte first) - for SC servos
  uint16_t unpack_uint16_be(const uint8_t* buffer) {
    return (buffer[0] << 8) | buffer[1];  // HIGH | LOW
  }

  // Pack 16-bit value into little-endian byte array (LOW byte first) - for STS servos
  void pack_uint16_le(uint8_t* buffer, uint16_t value) {
    buffer[0] = value & 0xFF;         // LOW byte
    buffer[1] = (value >> 8) & 0xFF;  // HIGH byte
  }
  
  // Unpack 16-bit value from little-endian byte array (LOW byte first) - for STS servos
  uint16_t unpack_uint16_le(const uint8_t* buffer) {
    return buffer[0] | (buffer[1] << 8);  // LOW | HIGH
  }

  // Convenience wrappers that use the configured servo type
  void pack_uint16(uint8_t* buffer, uint16_t value) {
    if (servo_type_ == ServoType::STS) {
      pack_uint16_le(buffer, value);
    } else {
      pack_uint16_be(buffer, value);
    }
  }

  uint16_t unpack_uint16(const uint8_t* buffer) {
    if (servo_type_ == ServoType::STS) {
      return unpack_uint16_le(buffer);
    } else {
      return unpack_uint16_be(buffer);
    }
  }

  // Runtime validation: Check if a register is valid for the current servo type
  static bool is_register_valid_for_type(Register reg, ServoType type) {
    uint8_t addr = to_byte(reg);

    // Handle address 48 conflict: SC=LOCK, STS=TORQUE_LIMIT_L
    if (addr == 48) {
      // Both servos can use address 48, but for different purposes
      // This is validated at the semantic level (which register name was used)
      return (reg == Register::lock_sc && type == ServoType::SC) ||
             (reg == Register::torque_limit_l && type == ServoType::STS);
    }

    switch(reg) {
      // STS-only registers
      case Register::ofs_l:
      case Register::ofs_h:
      case Register::mode:
      case Register::acc:
      case Register::torque_limit_h:
      case Register::lock_sts:
        return type == ServoType::STS;

      // Common registers (valid for both types)
      default:
        return true;
    }
  }

public:

  // Helper method to calculate checksum
  // Checksum formula: ~(ID + LENGTH + INSTRUCTION + PARAMETERS)
  static uint8_t calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction, 
                                      const uint8_t* parameters = nullptr, int parameter_count = 0) {
    uint8_t sum = id + length + instruction;
    for(int i = 0; i < parameter_count; i++) {
      sum += parameters[i];
    }
    return ~sum;
  }

  // Serial configuration
  void set_serial(HardwareSerial* serial) { bus_serial_ = serial; }
  
  // Set servo type (call this before any operations)
  void set_servo_type(ServoType type) { 
    servo_type_ = type;
  }
  
  // Error state accessors
  inline bool ok() const { return last_error_ == ServoError::none; }
  inline ServoError last_error() const { return last_error_; }
  inline void clear_error() { last_error_ = ServoError::none; }

  bool send_command(uint8_t id, uint8_t instruction, uint8_t* parameters = nullptr, int parameter_count = 0) {
    const int max_parameter_count = max_servo_count + 2;
    if (parameter_count > max_parameter_count) {
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    // clear the rx
    if (bus_serial_->available()) {
      Serial.println("Clearing extra rx before sending a command");
      while (bus_serial_->available()) {
        auto b = bus_serial_->read();
        Serial.print(b, 16);
      }
      Serial.println();
    }

    constexpr int MAX_PACKET_SIZE = MIN_PACKET_SIZE + max_servo_count;
    uint8_t packet[MAX_PACKET_SIZE];
    
    // Build packet
    packet[to_index(PacketOffset::header_1)] = to_byte(Protocol::header_byte_1);
    packet[to_index(PacketOffset::header_2)] = to_byte(Protocol::header_byte_2);
    packet[to_index(PacketOffset::id)] = id;
    packet[to_index(PacketOffset::length)] = INSTRUCTION_OVERHEAD + parameter_count;  // LENGTH = instruction byte + parameters
    packet[to_index(PacketOffset::instruction)] = instruction;
    
    // Copy parameters
    for(int i = 0; i < parameter_count; i++) {
      packet[to_index(PacketOffset::parameters) + i] = parameters[i];
    }
    
    // Calculate and append checksum
    packet[to_index(PacketOffset::parameters) + parameter_count] = calculate_checksum(id, packet[to_index(PacketOffset::length)], instruction, parameters, parameter_count);
    
    int packet_size = MIN_PACKET_SIZE + parameter_count;
    
    // Send packet
    bus_serial_->write(packet, packet_size);
    
    // Broadcast messages (ID 0xFE) don't produce echo, but still need to wait for transmission
    // Actually, they DO produce echo on half-duplex serial, we just don't wait for ACK responses
    bool is_broadcast = (id == to_byte(Protocol::broadcast_id));
    
    // Discard echo bytes as they arrive during transmission
    int echo_count = 0;
    unsigned long echo_start = millis();
    while(echo_count < packet_size) {
      if(bus_serial_->available()) {
        bus_serial_->read();
        echo_count++;
      }
      if(millis() - echo_start > timeout_ms_) {
        last_error_ = ServoError::timeout;
        return false;
      }
    }

    last_error_ = ServoError::none;
    return true;
  }

  bool read_response(uint8_t* response, int expected_size) {
    unsigned long start_ms = millis();;
    
    // Wait for expected response size
    while(bus_serial_->available() < expected_size && millis()-start_ms < timeout_ms_);
    
    if(bus_serial_->available() < expected_size) {
      last_error_ = ServoError::timeout;
      return false;
    }
    
    bus_serial_->readBytes(response, expected_size);
    
    // Validate header
    if(response[to_index(PacketOffset::header_1)] != to_byte(Protocol::header_byte_1) || response[to_index(PacketOffset::header_2)] != to_byte(Protocol::header_byte_2)) {
      last_error_ = ServoError::invalid_header;
      return false;
    }
    
    // Validate checksum
    int parameter_count = expected_size - MIN_PACKET_SIZE;
    uint8_t expected_checksum = calculate_checksum(
      response[to_index(PacketOffset::id)], 
      response[to_index(PacketOffset::length)], 
      response[to_index(PacketOffset::instruction)],
      parameter_count > 0 ? &response[to_index(PacketOffset::parameters)] : nullptr,
      parameter_count
    );
    
    if(expected_checksum != response[expected_size - 1]) {
      last_error_ = ServoError::checksum_mismatch;
      return false;
    }
    
    last_error_ = ServoError::none;
    return true;
  }

  std::optional<int> read_position(uint8_t servo_id) {
    uint8_t parameters[] = {to_byte(Register::present_position_l), 2};  // Read current position (2 bytes)
    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return std::nullopt;
    }

    uint8_t response[8];
    if(!read_response(response, 8)) {
      return std::nullopt;
    }

    // Extract position using configured byte order
    int position = unpack_uint16(&response[5]);
    last_error_ = ServoError::none;
    return position;
  }

  std::optional<int16_t> read_speed(uint8_t servo_id) {
    uint8_t parameters[] = {to_byte(Register::present_speed_l), 2};  // Read current speed (2 bytes)
    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return std::nullopt;
    }

    uint8_t response[8];
    if(!read_response(response, 8)) {
      return std::nullopt;
    }

    // Extract speed using configured byte order
    uint16_t raw_speed = unpack_uint16(&response[5]);
    
    // Speed format: bit 15 = direction (0=CW/positive, 1=CCW/negative)
    // Lower 15 bits = magnitude
    int16_t speed;
    if (raw_speed & (1 << 15)) {
      // Bit 15 set = CCW = negative
      speed = -(raw_speed & 0x7FFF);  // Mask off direction bit, negate
    } else {
      // Bit 15 clear = CW = positive
      speed = raw_speed & 0x7FFF;     // Mask off direction bit
    }
    
    last_error_ = ServoError::none;
    return speed;
  }

  std::optional<int16_t> read_load(uint8_t servo_id) {
    uint8_t parameters[] = {to_byte(Register::present_load_l), 2};  // Read current load/torque (2 bytes)
    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return std::nullopt;
    }

    uint8_t response[8];
    if(!read_response(response, 8)) {
      return std::nullopt;
    }

    // Extract load using configured byte order
    // Load is signed: positive=CW load, negative=CCW load
    int16_t load = static_cast<int16_t>(unpack_uint16(&response[5]));
    last_error_ = ServoError::none;
    return load;
  }

  std::optional<uint8_t> ping(uint8_t servo_id) {
    if(!send_command(servo_id, to_byte(Instruction::ping))) {  // Use ping instruction
      return std::nullopt;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      return std::nullopt;
    }
    
    if(response[to_index(PacketOffset::id)] != servo_id) {
      last_error_ = ServoError::invalid_response;
      return std::nullopt;
    }
    
    last_error_ = ServoError::none;
    return response[to_index(PacketOffset::id)];
  }

  bool write_position(uint8_t servo_id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    // Write 6 bytes starting at goal_position_l: position(2), time(2), speed(2)
    uint8_t parameters[7];
    parameters[0] = to_byte(Register::goal_position_l);
    // Use configured byte order
    pack_uint16(&parameters[1], position);
    pack_uint16(&parameters[3], time_ms);
    pack_uint16(&parameters[5], speed);

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 7)) {
      return false;
    }

    // Read ACK response
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Write position with hardware acceleration control - STS servos only
  // ACC parameter (0-255) controls velocity ramping for smooth motion
  bool write_position_sts_with_accel(uint8_t servo_id, uint16_t position, uint16_t speed, uint8_t acc) {
    // Safety check: ACC register only exists on STS servos
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Acceleration control only supported on STS servos");
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    // Write 7 bytes starting at ACC register: acc(1), position(2), time(2), speed(2)
    // Note: time_ms is set to 0 when using acceleration control (ACC controls ramp rate)
    uint8_t parameters[8];
    parameters[0] = to_byte(Register::acc);
    parameters[1] = acc;  // ACC value (0-255)
    // Use configured byte order (little-endian for STS)
    pack_uint16(&parameters[2], position);
    pack_uint16(&parameters[4], 0);      // time_ms = 0 when using ACC
    pack_uint16(&parameters[6], speed);

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 8)) {
      return false;
    }

    // Read ACK response
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Read a single byte from a register
  std::optional<uint8_t> read_byte(uint8_t servo_id, Register reg) {
    uint8_t parameters[] = {to_byte(reg), 1};  // Read 1 byte
    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return std::nullopt;
    }

    uint8_t response[7];  // MIN_PACKET_SIZE (6) + 1 data byte
    if(!read_response(response, 7)) {
      return std::nullopt;
    }

    last_error_ = ServoError::none;
    return response[5];  // Data byte at offset 5
  }

  // Write a single byte to a register
  bool write_byte(uint8_t servo_id, Register reg, uint8_t value) {
    // Validate register is appropriate for servo type
    if (!is_register_valid_for_type(reg, servo_type_)) {
      Serial.printf("ERROR: Register %d invalid for %s servo ID %d\n",
                    to_byte(reg),
                    servo_type_ == ServoType::SC ? "SC" : "STS",
                    servo_id);
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    uint8_t parameters[2];
    parameters[0] = to_byte(reg);
    parameters[1] = value;

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 2)) {
      return false;
    }

    // Read ACK response
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  bool write_id(uint8_t current_id, uint8_t new_id) {
    // Write new ID to EEPROM register 5
    uint8_t parameters[2];
    parameters[0] = to_byte(Register::id);
    parameters[1] = new_id;

    if(!send_command(current_id, to_byte(Instruction::write), parameters, 2)) {
      return false;
    }

    // Read ACK response
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    Serial.printf("Changed servo ID from %d to %d\n", current_id, new_id);
    last_error_ = ServoError::none;
    return true;
  }

  // Read identifying information from servo
  struct ServoInfo {
    uint16_t version;
    uint16_t min_angle;
    uint16_t max_angle;
  };

  std::optional<ServoInfo> read_info(uint8_t servo_id) {

    bool ok;

    ServoInfo info = {0, 0, 0};
    
    // Read version (registers 3-4, 2 bytes)
    uint8_t ver_params[] = {to_byte(Register::version_l), 2};
    ok = send_command(servo_id, to_byte(Instruction::read), ver_params, 2);
    if(!ok) return std::nullopt; 
    
    uint8_t ver_response[8];
    ok = read_response(ver_response, 8);
    if (!ok) return std::nullopt;
    info.version = unpack_uint16(&ver_response[5]);
    
    // Read min angle limit (registers 9-10, 2 bytes)
    uint8_t min_params[] = {to_byte(Register::min_angle_limit_l), 2};
    ok = send_command(servo_id, to_byte(Instruction::read), min_params, 2);
    if (!ok) return std::nullopt;

    uint8_t min_response[8];
    ok = read_response(min_response, 8);
    if (!ok) return std::nullopt;
    info.min_angle = unpack_uint16(&min_response[5]);
    
    // Read max angle limit (registers 11-12, 2 bytes)
    uint8_t max_params[] = {to_byte(Register::max_angle_limit_l), 2};
    ok = send_command(servo_id, to_byte(Instruction::read), max_params, 2);
    if (!ok) return std::nullopt;

    uint8_t max_response[8];
    ok = read_response(max_response, 8);
    if (!ok) return std::nullopt;
    info.max_angle = unpack_uint16(&max_response[5]);
    
    return info;
  }

  // Enable wheel mode (continuous rotation) for STS servos
  // Write 1 to MODE register (33) to enable velocity control mode
  bool enable_wheel_mode(uint8_t servo_id) {
    // Wheel mode is only supported on STS servos
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Wheel mode only supported on STS servos, not SC servos");
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    // First, enable torque
    uint8_t torque_params[2];
    torque_params[0] = to_byte(Register::torque_enable);
    torque_params[1] = 1;  // Enable

    if(!send_command(servo_id, to_byte(Instruction::write), torque_params, 2)) {
      return false;
    }

    uint8_t torque_response[MIN_PACKET_SIZE];
    if(!read_response(torque_response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    // Write 1 to MODE register to enable wheel mode (STS servos only)
    uint8_t mode_params[2];
    mode_params[0] = to_byte(Register::mode);
    mode_params[1] = 1;  // 1 = wheel mode

    if(!send_command(servo_id, to_byte(Instruction::write), mode_params, 2)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Restore position mode from wheel mode
  bool enable_position_mode(uint8_t servo_id, uint16_t min_angle, uint16_t max_angle) {
    if (servo_type_ == ServoType::STS) {
      // STS servos: Write 0 to MODE register to restore position mode
      uint8_t mode_params[2];
      mode_params[0] = to_byte(Register::mode);
      mode_params[1] = 0;  // 0 = position mode

      if(!send_command(servo_id, to_byte(Instruction::write), mode_params, 2)) {
        return false;
      }

      uint8_t response[MIN_PACKET_SIZE];
      if(!read_response(response, MIN_PACKET_SIZE)) {
        last_error_ = ServoError::no_ack;
        return false;
      }
    } else {
      // SC servos: Restore angle limits (they don't have wheel mode)
      // If this is called for SC servo, it's likely coming from PWM mode restore
      // Restore the angle limits that were saved before entering PWM mode
      uint8_t params[5];
      params[0] = to_byte(Register::min_angle_limit_l);

      // Pack min angle using big-endian for SC servos
      pack_uint16_be(&params[1], min_angle);

      // Pack max angle using big-endian for SC servos
      pack_uint16_be(&params[3], max_angle);

      if(!send_command(servo_id, to_byte(Instruction::write), params, 5)) {
        return false;
      }

      uint8_t response[MIN_PACKET_SIZE];
      if(!read_response(response, MIN_PACKET_SIZE)) {
        last_error_ = ServoError::no_ack;
        return false;
      }
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Set wheel velocity for continuous rotation (only works in wheel mode)
  // speed: signed 16-bit value where:
  //   - Positive = CW rotation, magnitude = speed
  //   - Negative = CCW rotation, magnitude = abs(speed)
  //   - Direction encoded in bit 15: 0=CW, 1=CCW
  bool set_wheel_velocity(uint8_t servo_id, int16_t speed) {
    uint16_t speed_value;

    if (speed < 0) {
      // CCW: Set bit 15 for direction, use absolute value for speed
      speed_value = (-speed) | (1 << 15);
    } else {
      // CW: Bit 15 = 0, use speed as-is
      speed_value = speed;
    }

    uint8_t parameters[3];
    parameters[0] = to_byte(Register::goal_speed_l);
    pack_uint16(&parameters[1], speed_value);

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 3)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Set offset calibration (STS servos only)
  // Offset range: typically -2048 to 2047 (16-bit signed value)
  bool set_offset(uint8_t servo_id, int16_t offset) {
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Offset calibration only supported on STS servos");
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    uint8_t parameters[3];
    parameters[0] = to_byte(Register::ofs_l);
    pack_uint16(&parameters[1], static_cast<uint16_t>(offset));

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 3)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Read offset calibration (STS servos only)
  int16_t read_offset(uint8_t servo_id) {
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Offset calibration only supported on STS servos");
      last_error_ = ServoError::invalid_parameter;
      return 0;
    }

    uint8_t parameters[2];
    parameters[0] = to_byte(Register::ofs_l);
    parameters[1] = 2;  // Read 2 bytes

    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return 0;
    }

    uint8_t response[MIN_PACKET_SIZE + 2];
    if(!read_response(response, MIN_PACKET_SIZE + 2)) {
      last_error_ = ServoError::timeout;
      return 0;
    }

    last_error_ = ServoError::none;
    return static_cast<int16_t>(unpack_uint16(&response[5]));
  }

  // Set torque limit (STS servos only)
  // Torque limit range: 0-1023 (default usually 1023 = 100%)
  bool set_torque_limit(uint8_t servo_id, uint16_t limit) {
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Torque limit only supported on STS servos");
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    if (limit > 1023) {
      Serial.println("ERROR: Torque limit must be 0-1023");
      last_error_ = ServoError::invalid_parameter;
      return false;
    }

    uint8_t parameters[3];
    parameters[0] = to_byte(Register::torque_limit_l);
    pack_uint16(&parameters[1], limit);

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 3)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Read torque limit (STS servos only)
  uint16_t read_torque_limit(uint8_t servo_id) {
    if (servo_type_ != ServoType::STS) {
      Serial.println("ERROR: Torque limit only supported on STS servos");
      last_error_ = ServoError::invalid_parameter;
      return 0;
    }

    uint8_t parameters[2];
    parameters[0] = to_byte(Register::torque_limit_l);
    parameters[1] = 2;  // Read 2 bytes

    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return 0;
    }

    uint8_t response[MIN_PACKET_SIZE + 2];
    if(!read_response(response, MIN_PACKET_SIZE + 2)) {
      last_error_ = ServoError::timeout;
      return 0;
    }

    last_error_ = ServoError::none;
    return unpack_uint16(&response[5]);
  }

  // Enable torque (both SC and STS servos)
  bool enable_torque(uint8_t servo_id) {
    uint8_t parameters[2];
    parameters[0] = to_byte(Register::torque_enable);
    parameters[1] = 1;  // Enable

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 2)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Disable torque (both SC and STS servos)
  bool disable_torque(uint8_t servo_id) {
    uint8_t parameters[2];
    parameters[0] = to_byte(Register::torque_enable);
    parameters[1] = 0;  // Disable

    if(!send_command(servo_id, to_byte(Instruction::write), parameters, 2)) {
      return false;
    }

    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::no_ack;
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Read torque enable status (both SC and STS servos)
  std::optional<bool> read_torque_enabled(uint8_t servo_id) {
    uint8_t parameters[2];
    parameters[0] = to_byte(Register::torque_enable);
    parameters[1] = 1;  // Read 1 byte

    if(!send_command(servo_id, to_byte(Instruction::read), parameters, 2)) {
      return std::nullopt;
    }

    uint8_t response[MIN_PACKET_SIZE + 1];
    if(!read_response(response, MIN_PACKET_SIZE + 1)) {
      last_error_ = ServoError::timeout;
      return std::nullopt;
    }

    last_error_ = ServoError::none;
    return response[5] != 0;
  }

  // Sync write positions to multiple servos at once
  bool sync_write_positions(const std::vector<uint8_t>& servo_ids, 
                            const std::vector<uint16_t>& positions,
                            const std::vector<uint16_t>& times,
                            const std::vector<uint16_t>& speeds) {
    if (servo_ids.size() != positions.size() || 
        servo_ids.size() != times.size() || 
        servo_ids.size() != speeds.size() ||
        servo_ids.empty() || 
        servo_ids.size() > max_servo_count) {
      last_error_ = ServoError::invalid_parameter;
      return false;
    }
    
    // Build sync_write packet
    // Packet: FF FF FE LENGTH INST_SYNC_WRITE MEMADDR DATALEN [ID1 DATA1...] [ID2 DATA2...] CHECKSUM
    // DATALEN = 6 (position(2) + time(2) + speed(2))
    constexpr uint8_t POSITION_DATA_SIZE = 6;  // position(2) + time(2) + speed(2)
    constexpr int SYNC_WRITE_HEADER_SIZE = 7;  // header(2) + id(1) + length(1) + instruction(1) + memaddr(1) + datalen(1)
    constexpr int CHECKSUM_SIZE = 1;
    constexpr int ID_SIZE = 1;
    constexpr int SYNC_WRITE_LENGTH_OVERHEAD = 4;  // instruction + memaddr + datalen + checksum (not including servo data)
    
    uint8_t packet_size = SYNC_WRITE_HEADER_SIZE + servo_ids.size() * (ID_SIZE + POSITION_DATA_SIZE) + CHECKSUM_SIZE;
    uint8_t packet[100];
    
    packet[to_index(PacketOffset::header_1)] = to_byte(Protocol::header_byte_1);
    packet[to_index(PacketOffset::header_2)] = to_byte(Protocol::header_byte_2);
    packet[to_index(PacketOffset::id)] = to_byte(Protocol::broadcast_id);
    packet[to_index(PacketOffset::length)] = (servo_ids.size() * (ID_SIZE + POSITION_DATA_SIZE)) + SYNC_WRITE_LENGTH_OVERHEAD;
    packet[to_index(PacketOffset::instruction)] = to_byte(Instruction::sync_write);
    packet[to_index(PacketOffset::parameters)] = to_byte(Register::goal_position_l);  // Memory address
    packet[to_index(PacketOffset::parameters) + 1] = POSITION_DATA_SIZE;  // bytes per servo
    
    uint8_t checksum = packet[to_index(PacketOffset::id)] + packet[to_index(PacketOffset::length)] + 
                       packet[to_index(PacketOffset::instruction)] + packet[to_index(PacketOffset::parameters)] + 
                       packet[to_index(PacketOffset::parameters) + 1];
    int offset = SYNC_WRITE_HEADER_SIZE;
    
    for (size_t i = 0; i < servo_ids.size(); i++) {
      packet[offset++] = servo_ids[i];
      checksum += servo_ids[i];
      
      // Pack using configured byte order
      pack_uint16(&packet[offset], positions[i]);
      pack_uint16(&packet[offset + 2], times[i]);
      pack_uint16(&packet[offset + 4], speeds[i]);
      
      for (int j = 0; j < POSITION_DATA_SIZE; j++) {
        checksum += packet[offset + j];
      }
      offset += POSITION_DATA_SIZE;
    }
    
    packet[offset] = ~checksum;
    
    // Send packet
    bus_serial_->write(packet, packet_size);
    
    // Consume echo
    int echo_count = 0;
    unsigned long echo_start = millis();
    while(echo_count < packet_size) {
      if(bus_serial_->available()) {
        bus_serial_->read();
        echo_count++;
      }
      if(millis() - echo_start > timeout_ms_) {
        last_error_ = ServoError::timeout;
        return false;
      }
    }
    
    last_error_ = ServoError::none;
    return true;
  }

  // Sync read positions from multiple servos at once
  // NOTE: SCSCL servos (SC15, SC09, etc.) do NOT support sync_read instruction
  // This is only supported by SMS_STS and HLSCL series servos
  // For SC servos, use individual reads
  std::vector<std::optional<int>> sync_read_positions(const std::vector<uint8_t>& servo_ids) {
    std::vector<std::optional<int>> results(servo_ids.size());
    
    if (servo_ids.empty() || servo_ids.size() > max_servo_count) {
      last_error_ = ServoError::invalid_parameter;
      return results;
    }
    
    Serial.printf("Note: SCSCL servos don't support sync_read, using individual reads\n");
    Serial.printf("Reading positions from %d servos...\n", servo_ids.size());
    
    // Use individual reads for SC series servos
    for (size_t i = 0; i < servo_ids.size(); i++) {
      results[i] = read_position(servo_ids[i]);
    }
    
    last_error_ = ServoError::none;
    return results;
  }

} servo_bus;

// Base servo class
class Servo {
protected:
  SCServoBus* bus_;
  uint8_t id_;
  uint16_t min_encoder_angle_ = 0;
  uint16_t max_encoder_angle_ = 4095;
  bool info_loaded_ = false;

public:
  Servo(SCServoBus* bus, uint8_t id) : bus_(bus), id_(id) {}
  virtual ~Servo() = default;
  
  uint8_t id() const { return id_; }
  uint16_t min_encoder_angle() const { return min_encoder_angle_; }
  uint16_t max_encoder_angle() const { return max_encoder_angle_; }
  uint16_t encoder_angle_range() const { return max_encoder_angle_ - min_encoder_angle_; }
  bool info_loaded() const { return info_loaded_; }
  
  virtual SCServoBus::ServoType type() const = 0;
  
  std::optional<SCServoBus::ServoInfo> read_info() {
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

    if (type() == SCServoBus::ServoType::STS) {
      // STS servos: Set MODE register to 2 for PWM open-loop mode
      uint8_t mode_params[2];
      mode_params[0] = SCServoBus::to_byte(SCServoBus::Register::mode);
      mode_params[1] = 2;  // Mode 2 = PWM open-loop speed regulation

      if(!bus_->send_command(id_, SCServoBus::to_byte(SCServoBus::Instruction::write), mode_params, 2)) {
        return false;
      }

      uint8_t response[SCServoBus::MIN_PACKET_SIZE];
      if(!bus_->read_response(response, SCServoBus::MIN_PACKET_SIZE)) {
        return false;
      }
    } else {
      // SC servos: Set min and max angle limits to 0 to enable PWM mode
      uint8_t params[5];
      params[0] = SCServoBus::to_byte(SCServoBus::Register::min_angle_limit_l);
      params[1] = 0;  // min_angle_l
      params[2] = 0;  // min_angle_h
      params[3] = 0;  // max_angle_l
      params[4] = 0;  // max_angle_h

      if(!bus_->send_command(id_, SCServoBus::to_byte(SCServoBus::Instruction::write), params, 5)) {
        return false;
      }

      uint8_t response[SCServoBus::MIN_PACKET_SIZE];
      if(!bus_->read_response(response, SCServoBus::MIN_PACKET_SIZE)) {
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
    parameters[0] = SCServoBus::to_byte(SCServoBus::Register::goal_time_l);

    // Pack using configured byte order
    uint8_t temp[2];
    bus_->set_servo_type(type());
    if (type() == SCServoBus::ServoType::STS) {
      temp[0] = pwm_value & 0xFF;
      temp[1] = (pwm_value >> 8) & 0xFF;
    } else {
      temp[0] = (pwm_value >> 8) & 0xFF;
      temp[1] = pwm_value & 0xFF;
    }
    parameters[1] = temp[0];
    parameters[2] = temp[1];

    if(!bus_->send_command(id_, SCServoBus::to_byte(SCServoBus::Instruction::write), parameters, 3)) {
      return false;
    }

    uint8_t response[SCServoBus::MIN_PACKET_SIZE];
    return bus_->read_response(response, SCServoBus::MIN_PACKET_SIZE);
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

// SC series servo (big-endian)
class SCServo : public Servo {
public:
  using Servo::Servo;
  SCServoBus::ServoType type() const override { return SCServoBus::ServoType::SC; }
};

// STS series servo (little-endian)
class STSServo : public Servo {
public:
  using Servo::Servo;
  SCServoBus::ServoType type() const override { return SCServoBus::ServoType::STS; }
  
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

void scan_ids(uint32_t start_id=1, uint32_t end_id=255) {
  Serial.printf("scanning for servo ids from %d to %d\n", start_id, end_id);
  for (uint32_t id = start_id; id<= end_id; ++id) {
    if (servo_bus.ping(id)) {
      Serial.printf("found servo with id %d\n",id);
    }
  }
  Serial.println("done");
}

// Diagnostic function to check EEPROM lock status and ID configuration
void diagnose_servo(uint8_t servo_id, SCServoBus::ServoType type) {
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
  auto id_value = servo_bus.read_byte(servo_id, SCServoBus::Register::id);
  if (id_value) {
    Serial.printf("✓ ID register value: %d\n", *id_value);
  } else {
    Serial.printf("✗ Failed to read ID register\n");
  }

  // Read LOCK register status (use correct register for servo type)
  SCServoBus::Register lock_reg = (type == SCServoBus::ServoType::STS) ?
                                   SCServoBus::Register::lock_sts :
                                   SCServoBus::Register::lock_sc;
  uint8_t lock_reg_num = (type == SCServoBus::ServoType::STS) ? 55 : 48;

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

// Set servo ID permanently by unlocking EEPROM, writing, and re-locking
bool set_servo_id_permanent(uint8_t current_id, uint8_t new_id, SCServoBus::ServoType type) {
  Serial.printf("\n=== Setting Servo ID: %d -> %d (Permanent) ===\n", current_id, new_id);

  servo_bus.set_servo_type(type);

  // Determine correct LOCK register for this servo type
  SCServoBus::Register lock_reg = (type == SCServoBus::ServoType::STS) ?
                                   SCServoBus::Register::lock_sts :
                                   SCServoBus::Register::lock_sc;
  uint8_t lock_reg_num = (type == SCServoBus::ServoType::STS) ? 55 : 48;

  Serial.printf("Using LOCK register %d for %s servo\n", lock_reg_num,
                (type == SCServoBus::ServoType::STS) ? "STS" : "SC");

  // Step 1: Verify servo responds
  Serial.printf("Step 1: Pinging servo at current ID %d...\n", current_id);
  if (!servo_bus.ping(current_id)) {
    Serial.printf("✗ ERROR: Servo does not respond at ID %d\n", current_id);
    return false;
  }
  Serial.println("✓ Servo responds");

  // Step 2: Read current LOCK status
  Serial.println("Step 2: Reading LOCK register...");
  auto initial_lock = servo_bus.read_byte(current_id, lock_reg);
  if (!initial_lock) {
    Serial.println("✗ ERROR: Failed to read LOCK register");
    return false;
  }
  Serial.printf("✓ Current LOCK value: %d %s\n", *initial_lock,
                (*initial_lock == 1) ? "(LOCKED)" : "(UNLOCKED)");

  // Step 3: Unlock EEPROM
  if (*initial_lock != 0) {
    Serial.println("Step 3: Unlocking EEPROM (writing 0 to LOCK register)...");
    if (!servo_bus.write_byte(current_id, lock_reg, 0)) {
      Serial.println("✗ ERROR: Failed to unlock EEPROM");
      return false;
    }
    Serial.println("✓ EEPROM unlocked");

    // Verify unlock
    auto verify_unlock = servo_bus.read_byte(current_id, lock_reg);
    if (!verify_unlock || *verify_unlock != 0) {
      Serial.println("✗ ERROR: EEPROM unlock verification failed");
      return false;
    }
    Serial.println("✓ Unlock verified");
    delay(100);  // Give EEPROM time to fully unlock
  } else {
    Serial.println("Step 3: EEPROM already unlocked, skipping");
  }

  // Step 4: Write new ID
  // NOTE: Writing the ID register is special - the servo changes its ID immediately,
  // so we can't use the normal write_byte which expects an ACK from the old ID.
  // We need to send the command but not wait for ACK (or ignore it).
  Serial.printf("Step 4: Writing new ID %d to register 5...\n", new_id);

  uint8_t id_params[2];
  id_params[0] = SCServoBus::to_byte(SCServoBus::Register::id);
  id_params[1] = new_id;

  if (!servo_bus.send_command(current_id, SCServoBus::to_byte(SCServoBus::Instruction::write), id_params, 2)) {
    Serial.println("✗ ERROR: Failed to send ID write command");
    // Try to re-lock before returning
    servo_bus.write_byte(new_id, lock_reg, 1);  // Use new_id since servo may have changed
    return false;
  }

  // The servo has now changed its ID, so clear any response bytes
  delay(100);  // Give servo time to process and send ACK
  while (Serial1.available()) {
    Serial1.read();  // Discard ACK bytes
  }

  Serial.println("✓ New ID write command sent");

  // Step 5: Verify ID write (read back from EEPROM)
  Serial.println("Step 5: Verifying ID write...");
  delay(50);  // Give EEPROM time to write
  // NOTE: After writing ID, servo now responds to NEW ID, not old ID
  auto verify_id = servo_bus.read_byte(new_id, SCServoBus::Register::id);
  if (!verify_id || *verify_id != new_id) {
    Serial.printf("✗ ERROR: ID verification failed (expected %d, got %d)\n",
                  new_id, verify_id ? *verify_id : 0);
    // Try to re-lock before returning (use new_id since servo changed ID)
    servo_bus.write_byte(new_id, lock_reg, 1);
    return false;
  }
  Serial.printf("✓ ID verified: %d\n", *verify_id);

  // Step 6: Re-lock EEPROM (best practice)
  Serial.println("Step 6: Re-locking EEPROM (writing 1 to LOCK register)...");
  // NOTE: We now need to use the NEW ID since the servo has changed its ID
  if (!servo_bus.write_byte(new_id, lock_reg, 1)) {
    Serial.println("⚠ WARNING: Failed to re-lock EEPROM");
    Serial.println("  ID change succeeded, but EEPROM remains unlocked");
    Serial.println("  Consider manually locking it for safety");
  } else {
    Serial.println("✓ EEPROM re-locked");

    // Verify lock
    auto verify_lock = servo_bus.read_byte(new_id, lock_reg);
    if (verify_lock && *verify_lock == 1) {
      Serial.println("✓ Lock verified");
    }
  }

  // Success message
  Serial.println("\n=== SUCCESS ===");
  Serial.printf("Servo ID changed from %d to %d\n", current_id, new_id);
  Serial.println("✓ Change is permanent (written to EEPROM)");
  Serial.println("\nIMPORTANT: Power cycle the servo to confirm the ID persists!");
  Serial.println("Then run scan_ids(1, 10) to verify.\n");

  return true;
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
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
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
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
  servo_bus.sync_write_positions(sc_ids, sc_maxs, sc_times, sc_speeds);
  
  // Move STS servo to its max position
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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
    servo_bus.set_servo_type(SCServoBus::ServoType::SC);
    for (auto id : sc_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    
    // Read STS servo
    servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
  servo_bus.sync_write_positions(sc_ids, sc_mins, sc_times, sc_speeds);
  
  // Move STS servo back to its min position
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
  servo_bus.sync_write_positions(sts_ids, sts_mins, sts_times, sts_speeds);
  
  Serial.println("\nMonitoring positions (reading 2x per second):");
  Serial.println("Time(ms)  | Servo #1 | Servo #2 | Servo #3 | Servo #4");
  Serial.println("----------|----------|----------|----------|----------");
  
  start_time = millis();
  
  while (millis() - start_time < duration) {
    unsigned long t = millis() - start_time;
    Serial.printf("%8lu  |", t);
    
    // Read SC servos
    servo_bus.set_servo_type(SCServoBus::ServoType::SC);
    for (auto id : sc_ids) {
      auto pos = servo_bus.read_position(id);
      if (pos) {
        Serial.printf(" %8d |", *pos);
      } else {
        Serial.printf("     FAIL |");
      }
    }
    
    // Read STS servo
    servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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
  scan_ids(1, 10);
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
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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
  
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
  
  // Step 3: Try to reset mode on ID 252 (appears to be broadcasting)
  Serial.println("Attempting to reset servo ID 252 mode...");
  uint8_t mode_params[2];
  mode_params[0] = 33;  // SMS_STS_MODE register
  mode_params[1] = 0;   // Set to normal servo mode (not wheel mode)
  servo_bus.send_command(252, SCServoBus::to_byte(SCServoBus::Instruction::write), mode_params, 2);
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
  torque_params[0] = SCServoBus::to_byte(SCServoBus::Register::torque_enable);
  torque_params[1] = 0;  // Disable
  servo_bus.send_command(0xFE, SCServoBus::to_byte(SCServoBus::Instruction::write), torque_params, 2);
  delay(100);
  
  // Step 6: Try to set torque limit to 0 (ID 252)
  Serial.println("Setting torque limit to 0...");
  uint8_t torque_limit_params[3];
  torque_limit_params[0] = 48;  // SMS_STS_TORQUE_LIMIT_L
  torque_limit_params[1] = 0;   // LOW byte
  torque_limit_params[2] = 0;   // HIGH byte
  servo_bus.send_command(252, SCServoBus::to_byte(SCServoBus::Instruction::write), torque_limit_params, 3);
  delay(100);
  
  // Step 7: Try specific servo IDs (1-4)
  Serial.println("Trying individual servo IDs 1-4...");
  for (uint8_t id = 1; id <= 4; id++) {
    // Set mode to 0 (servo mode)
    mode_params[0] = 33;  // MODE
    mode_params[1] = 0;   // Normal mode
    servo_bus.send_command(id, SCServoBus::to_byte(SCServoBus::Instruction::write), mode_params, 2);
    delay(30);
    
    // Disable torque
    torque_params[0] = 40;  // TORQUE_ENABLE
    torque_params[1] = 0;   // Disable
    servo_bus.send_command(id, SCServoBus::to_byte(SCServoBus::Instruction::write), torque_params, 2);
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

  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
  
  // Step 1: Unlock EPROM (LOCK register = 0)
  Serial.println("\n═══ STEP 1: Unlocking EPROM ═══");
  uint8_t unlock_params[2];
  unlock_params[0] = SCServoBus::to_byte(SCServoBus::Register::lock_sts);  // Register 55
  unlock_params[1] = 0;  // Unlock
  
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::write), unlock_params, 2)) {
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
  limits_params[0] = SCServoBus::to_byte(SCServoBus::Register::min_angle_limit_l);  // Register 9
  limits_params[1] = 0x00;  // min LOW byte = 0
  limits_params[2] = 0x00;  // min HIGH byte = 0
  limits_params[3] = 0xFF;  // max LOW byte = 255
  limits_params[4] = 0x0F;  // max HIGH byte = 15 (0x0FFF = 4095)
  
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::write), limits_params, 5)) {
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
  lock_params[0] = SCServoBus::to_byte(SCServoBus::Register::lock_sts);  // Register 55
  lock_params[1] = 1;  // Lock
  
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::write), lock_params, 2)) {
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
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
  
  // Read min angle limit (registers 9-10, 2 bytes)
  Serial.println("\nReading MIN_ANGLE_LIMIT (registers 9-10, 2 bytes)...");
  uint8_t sc_min_params[] = {9, 2};
  if (servo_bus.send_command(4, SCServoBus::to_byte(SCServoBus::Instruction::read), sc_min_params, 2)) {
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
  if (servo_bus.send_command(4, SCServoBus::to_byte(SCServoBus::Instruction::read), sc_max_params, 2)) {
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
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
  
  // Read MODE register (register 33, 1 byte)
  Serial.println("\nReading MODE register (register 33, 1 byte)...");
  uint8_t sts_mode_params[] = {33, 1};
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::read), sts_mode_params, 2)) {
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
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::read), sts_min_params, 2)) {
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
  if (servo_bus.send_command(5, SCServoBus::to_byte(SCServoBus::Instruction::read), sts_max_params, 2)) {
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
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
  servo_bus.enable_torque(4);
  servo_bus.set_servo_type(SCServoBus::ServoType::STS);
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

void setup() {
  Serial.begin(1000000);
  delay(2000);

  Serial1.begin(1000000, SERIAL_8N1, pin_servo_rx, pin_servo_tx);
  servo_bus.set_serial(&Serial1);

  // Wait a bit after serial initialization
  delay(500);

  // === ID CONFIGURATION TOOLS ===
  // Uncomment these to diagnose and fix servo ID issues

  // 1. Diagnose a servo to check EEPROM lock status
  //    (Checks if EEPROM is locked, which prevents ID changes from persisting)
  // diagnose_servo(1, SCServoBus::ServoType::STS);  // Check STS servo at ID 1

  // 2. Set servo ID permanently (unlocks EEPROM, writes ID, re-locks)
  //    Use this to change servo IDs - changes will persist after power cycle
  // set_servo_id_permanent(1, 5, SCServoBus::ServoType::STS);  // Change STS servo from ID 1 to 5
  // set_servo_id_permanent(1, 2, SCServoBus::ServoType::SC);   // Change SC servo from ID 1 to 2

  // === SERVO ID SCHEME ===
  // SC servos: IDs 2, 3, 4 (big-endian)
  // STS servo: ID 5 (little-endian)
  // ID 1 = unconfigured/bad servo indicator

  // === DEMONSTRATIONS ===

  // Scan for all servos on the bus (1-255)
  scan_ids();

  // === ID CONFIGURATION EXAMPLES ===
  // Uncomment these lines to set up your servos with the new ID scheme:

  // 1. Diagnose a servo first to check its lock status
  // diagnose_servo(1, SCServoBus::ServoType::STS);  // For STS servo at ID 1
  // diagnose_servo(1, SCServoBus::ServoType::SC);   // For SC servo at ID 1

  // 2. Set permanent IDs (changes persist after power cycle)
  // set_servo_id_permanent(1, 5, SCServoBus::ServoType::STS);  // STS servo: 1 -> 5
  // set_servo_id_permanent(1, 2, SCServoBus::ServoType::SC);   // SC servo: 1 -> 2
  // set_servo_id_permanent(1, 3, SCServoBus::ServoType::SC);   // SC servo: 1 -> 3
  // set_servo_id_permanent(1, 4, SCServoBus::ServoType::SC);   // SC servo: 1 -> 4

  // 3. After setting IDs, power cycle each servo and scan again to verify



  // === ACTIVE TEST ===
  // diagnose_eprom_registers();           // Check what's actually in the servo EPROM
  // restore_sts_eprom();                  // Fix the corrupted STS servo EPROM
  test_speed_and_load_monitoring();     // Comprehensive speed & load test

  // Other demos available (currently not running):
  // emergency_servo_reset();              // Emergency recovery for stuck servos
  // demonstrate_pwm_mode();               // PWM mode demo - works on all servos
  // demonstrate_coordinated_moving();     // Original version with manual type switching
  // demonstrate_coordinated_moving_2();   // Clean version using servo objects
  // demonstrate_sts_features();           // Full STS features demo
  // demonstrate_wheel_mode();             // STS-only wheel mode demo
}

void loop() {}