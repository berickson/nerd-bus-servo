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
    mode = 33,  // Servo mode: 0=position, 1=wheel (continuous rotation)
    
    // SRAM (read/write)
    torque_enable = 40,
    acc = 41,  // Acceleration control (0-255) - STS servos only
    goal_position_l = 42,
    goal_position_h = 43,
    goal_time_l = 44,
    goal_time_h = 45,
    goal_speed_l = 46,
    goal_speed_h = 47,
    lock_sc = 48,    // LOCK register for SC servos (SCSCL)
    lock_sts = 55,   // LOCK register for STS servos (SMS_STS)
    
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
    bool valid;
  };

  ServoInfo read_info(uint8_t servo_id) {
    ServoInfo info = {0, 0, 0, false};
    
    // Read version (registers 3-4, 2 bytes)
    uint8_t ver_params[] = {to_byte(Register::version_l), 2};
    if(send_command(servo_id, to_byte(Instruction::read), ver_params, 2)) {
      uint8_t ver_response[8];
      if(read_response(ver_response, 8)) {
        info.version = unpack_uint16(&ver_response[5]);
      }
    }
    
    // Read min angle limit (registers 9-10, 2 bytes)
    uint8_t min_params[] = {to_byte(Register::min_angle_limit_l), 2};
    if(send_command(servo_id, to_byte(Instruction::read), min_params, 2)) {
      uint8_t min_response[8];
      if(read_response(min_response, 8)) {
        info.min_angle = unpack_uint16(&min_response[5]);
      }
    }
    
    // Read max angle limit (registers 11-12, 2 bytes)
    uint8_t max_params[] = {to_byte(Register::max_angle_limit_l), 2};
    if(send_command(servo_id, to_byte(Instruction::read), max_params, 2)) {
      uint8_t max_response[8];
      if(read_response(max_response, 8)) {
        info.max_angle = unpack_uint16(&max_response[5]);
        info.valid = true;
      }
    }
    
    return info;
  }

  // Enable wheel mode (continuous rotation) for STS servos
  // Write 1 to MODE register (33) to enable velocity control mode
  bool enable_wheel_mode(uint8_t servo_id) {
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
    
    // Write 1 to MODE register to enable wheel mode
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

  // Restore position mode by setting MODE back to 0
  bool restore_position_mode(uint8_t servo_id, uint16_t min_angle, uint16_t max_angle) {
    // Write 0 to MODE register to restore position mode
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
  
  bool read_info() {
    bus_->set_servo_type(type());
    auto info = bus_->read_info(id_);
    if (info.valid) {
      min_encoder_angle_ = info.min_angle;
      max_encoder_angle_ = info.max_angle;
      info_loaded_ = true;
    }
    return info.valid;
  }
  
  std::optional<int> read_encoder_angle() {
    bus_->set_servo_type(type());
    return bus_->read_position(id_);
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

  // Restore position mode after PWM mode
  bool restore_position_mode_from_pwm() {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());

    if (type() == SCServoBus::ServoType::STS) {
      // STS servos: Set MODE register back to 0 for position servo mode
      uint8_t mode_params[2];
      mode_params[0] = SCServoBus::to_byte(SCServoBus::Register::mode);
      mode_params[1] = 0;  // Mode 0 = Position servo mode

      if(!bus_->send_command(id_, SCServoBus::to_byte(SCServoBus::Instruction::write), mode_params, 2)) {
        return false;
      }

      uint8_t response[SCServoBus::MIN_PACKET_SIZE];
      if(!bus_->read_response(response, SCServoBus::MIN_PACKET_SIZE)) {
        return false;
      }
    } else {
      // SC servos: Restore angle limits to their original values
      uint8_t params[5];
      params[0] = SCServoBus::to_byte(SCServoBus::Register::min_angle_limit_l);

      // Pack min/max angle limits
      bus_->set_servo_type(type());
      if (type() == SCServoBus::ServoType::STS) {
        params[1] = min_encoder_angle_ & 0xFF;
        params[2] = (min_encoder_angle_ >> 8) & 0xFF;
        params[3] = max_encoder_angle_ & 0xFF;
        params[4] = (max_encoder_angle_ >> 8) & 0xFF;
      } else {
        params[1] = (min_encoder_angle_ >> 8) & 0xFF;
        params[2] = min_encoder_angle_ & 0xFF;
        params[3] = (max_encoder_angle_ >> 8) & 0xFF;
        params[4] = max_encoder_angle_ & 0xFF;
      }

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
  
  bool restore_position_mode() {
    if (!info_loaded_) return false;
    bus_->set_servo_type(type());
    bool result = bus_->restore_position_mode(id_, min_encoder_angle_, max_encoder_angle_);
    if (result) {
      // Re-read info to ensure cached values are correct
      read_info();
    }
    return result;
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
  if (info.valid) {
    Serial.printf("✓ Firmware version: %d\n", info.version);
    Serial.printf("✓ Angle limits: min=%d, max=%d\n", info.min_angle, info.max_angle);
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
    if (info.valid) {
      Serial.printf("  Servo #%d: min=%d, max=%d\n", id, info.min_angle, info.max_angle);
      sc_mins.push_back(info.min_angle);
      sc_maxs.push_back(info.max_angle);
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
    if (info.valid) {
      Serial.printf("  Servo #%d: min=%d, max=%d\n", id, info.min_angle, info.max_angle);
      sts_mins.push_back(info.min_angle);
      sts_maxs.push_back(info.max_angle);
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
  if (!servo_bus.restore_position_mode(sts_servo.id(), saved_min, saved_max)) {
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
  if (!sts_servo.restore_position_mode()) {
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

  

  // Other demos available (currently not running):
  // emergency_servo_reset();              // Emergency recovery for stuck servos
  demonstrate_pwm_mode();               // PWM mode demo - works on all servos
  // demonstrate_coordinated_moving();     // Original version with manual type switching
  // demonstrate_coordinated_moving_2();   // Clean version using servo objects
  // demonstrate_sts_features();           // Full STS features demo
  // demonstrate_wheel_mode();             // STS-only wheel mode demo
}

void loop() {}