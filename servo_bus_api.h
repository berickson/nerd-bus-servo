#pragma once
#include <map>
// this class exposes APIs for low level SC and STS bus servo protocol
class ServoBusApi {
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
  uint32_t timeout_ms_ = 2;
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

  // Helper to consume echo bytes after transmission
  bool consume_echo(int packet_size) {
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
    return true;
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

  bool send_command(uint8_t servo_id, Instruction instruction, uint8_t* parameters = nullptr, int parameter_count = 0) {
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
    packet[to_index(PacketOffset::id)] = servo_id;
    packet[to_index(PacketOffset::length)] = INSTRUCTION_OVERHEAD + parameter_count;  // LENGTH = instruction byte + parameters
    packet[to_index(PacketOffset::instruction)] = to_byte(instruction);
    
    // Copy parameters
    for(int i = 0; i < parameter_count; i++) {
      packet[to_index(PacketOffset::parameters) + i] = parameters[i];
    }
    
    // Calculate and append checksum
    packet[to_index(PacketOffset::parameters) + parameter_count] = calculate_checksum(servo_id, packet[to_index(PacketOffset::length)], to_byte(instruction), parameters, parameter_count);
    
    int packet_size = MIN_PACKET_SIZE + parameter_count;
    
    // Send packet
    bus_serial_->write(packet, packet_size);
    
    // Discard echo bytes as they arrive during transmission
    if (!consume_echo(packet_size)) {
      return false;
    }

    last_error_ = ServoError::none;
    return true;
  }

  // Set servo ID permanently by unlocking EEPROM, writing, and re-locking
  bool set_servo_id_permanent(uint8_t current_id, uint8_t new_id) {
    Serial.printf("\n=== Setting Servo ID: %d -> %d (Permanent) ===\n", current_id, new_id);

    // Determine correct LOCK register for this servo type
    ServoBusApi::Register lock_reg = (servo_type_ == ServoBusApi::ServoType::STS) ?
                                    ServoBusApi::Register::lock_sts :
                                    ServoBusApi::Register::lock_sc;
    uint8_t lock_reg_num = (servo_type_ == ServoBusApi::ServoType::STS) ? 55 : 48;

    Serial.printf("Using LOCK register %d for %s servo\n", lock_reg_num,
                  (servo_type_ == ServoBusApi::ServoType::STS) ? "STS" : "SC");

    // Step 1: Verify servo responds
    Serial.printf("Step 1: Pinging servo at current ID %d...\n", current_id);
    if (!ping(current_id)) {
      Serial.printf("✗ ERROR: Servo does not respond at ID %d\n", current_id);
      return false;
    }
    Serial.println("✓ Servo responds");

    // Step 2: Read current LOCK status
    Serial.println("Step 2: Reading LOCK register...");
    auto initial_lock = read_byte(current_id, lock_reg);
    if (!initial_lock) {
      Serial.println("✗ ERROR: Failed to read LOCK register");
      return false;
    }
    Serial.printf("✓ Current LOCK value: %d %s\n", *initial_lock,
                  (*initial_lock == 1) ? "(LOCKED)" : "(UNLOCKED)");

    // Step 3: Unlock EEPROM
    if (*initial_lock != 0) {
      Serial.println("Step 3: Unlocking EEPROM (writing 0 to LOCK register)...");
      if (!write_byte(current_id, lock_reg, 0)) {
        Serial.println("✗ ERROR: Failed to unlock EEPROM");
        return false;
      }
      Serial.println("✓ EEPROM unlocked");

      // Verify unlock
      auto verify_unlock = read_byte(current_id, lock_reg);
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
    id_params[0] = to_byte(Register::id);
    id_params[1] = new_id;

    if (!send_command(current_id, Instruction::write, id_params, 2)) {
      Serial.println("✗ ERROR: Failed to send ID write command");
      // Try to re-lock before returning
      write_byte(new_id, lock_reg, 1);  // Use new_id since servo may have changed
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
    auto verify_id = read_byte(new_id, ServoBusApi::Register::id);
    if (!verify_id || *verify_id != new_id) {
      Serial.printf("✗ ERROR: ID verification failed (expected %d, got %d)\n",
                    new_id, verify_id ? *verify_id : 0);
      // Try to re-lock before returning (use new_id since servo changed ID)
      write_byte(new_id, lock_reg, 1);
      return false;
    }
    Serial.printf("✓ ID verified: %d\n", *verify_id);

    // Step 6: Re-lock EEPROM (best practice)
    Serial.println("Step 6: Re-locking EEPROM (writing 1 to LOCK register)...");
    // NOTE: We now need to use the NEW ID since the servo has changed its ID
    if (!write_byte(new_id, lock_reg, 1)) {
      Serial.println("⚠ WARNING: Failed to re-lock EEPROM");
      Serial.println("  ID change succeeded, but EEPROM remains unlocked");
      Serial.println("  Consider manually locking it for safety");
    } else {
      Serial.println("✓ EEPROM re-locked");

      // Verify lock
      auto verify_lock = read_byte(new_id, lock_reg);
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

  // reads byte_count bytes from servo register table, starting at address given by register_id
  // returns a pointer to byte_count bytes that is only valid until the next api function call
  uint8_t * read_register(uint8_t servo_id, Register register_id, uint8_t byte_count) {
    static uint8_t buffer[255];
    uint8_t parameters[] = {to_byte(register_id), byte_count};  // Read current position (2 bytes)
    if(!send_command(servo_id, Instruction::read, parameters, 2)) {
      return nullptr;
    }

    if(!read_response(buffer, 6+byte_count)) {
      return nullptr;
    }

    return &buffer[5];
  }

  std::optional<uint8_t> read_voltage(uint8_t servo_id) {
    auto temperature = read_register(servo_id, Register::present_voltage, 1);
    if (temperature) {
      return *temperature;
    }
    return std::nullopt;

  }

  std::optional<uint8_t> read_temperature(uint8_t servo_id) {
    auto temperature = read_register(servo_id, Register::present_temperature, 1);
    if (temperature) {
      return *temperature;
    }
    return std::nullopt;

  }



  std::optional<int> read_position(uint8_t servo_id) {
    auto bytes = read_register(servo_id, Register::present_position_l, 2);
    if (!bytes) {
      return std::nullopt;
    }
    // Extract position using configured byte order
    int position = unpack_uint16(bytes);
    last_error_ = ServoError::none;
    return position;
  }

  std::optional<int16_t> read_speed(uint8_t servo_id) {
    auto bytes = read_register(servo_id, Register::present_speed_l, 2);
    if(!bytes) {
      return std::nullopt;
    }

    // Extract speed using configured byte order
    uint16_t raw_speed = unpack_uint16(bytes);
    
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
    auto bytes = read_register(servo_id, Register::present_load_l, 2); 
    if(!bytes) {
      return std::nullopt;
    }

    // Extract load using configured byte order
    // Load is signed: positive=CW load, negative=CCW load
    int16_t load = static_cast<int16_t>(unpack_uint16(bytes));
    last_error_ = ServoError::none;
    return load;
  }

  std::optional<uint8_t> ping(uint8_t servo_id) {
    if(!send_command(servo_id, Instruction::ping)) {  // Use ping instruction
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

    if(!send_command(servo_id, Instruction::write, parameters, 7)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 8)) {
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
    if(!send_command(servo_id, Instruction::read, parameters, 2)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 2)) {
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

    if(!send_command(current_id, Instruction::write, parameters, 2)) {
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

  // Read angle limits from servo
  struct AngleLimits {
    uint16_t min_angle;
    uint16_t max_angle;
  };

  std::optional<AngleLimits> read_angle_limits(uint8_t servo_id) {
    AngleLimits limits = {0, 0};
    
    // Read min angle limit (registers 9-10, 2 bytes)
    auto min_bytes = read_register(servo_id, Register::min_angle_limit_l, 2);
    if (!min_bytes) {
      return std::nullopt;
    }
    limits.min_angle = unpack_uint16(min_bytes);
    
    // Read max angle limit (registers 11-12, 2 bytes)
    auto max_bytes = read_register(servo_id, Register::max_angle_limit_l, 2);
    if (!max_bytes) {
      return std::nullopt;
    }
    limits.max_angle = unpack_uint16(max_bytes);
    
    last_error_ = ServoError::none;
    return limits;
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
    ok = send_command(servo_id, Instruction::read, ver_params, 2);
    if(!ok) return std::nullopt; 
    
    uint8_t ver_response[8];
    ok = read_response(ver_response, 8);
    if (!ok) return std::nullopt;
    info.version = unpack_uint16(&ver_response[5]);
    
    // Read min angle limit (registers 9-10, 2 bytes)
    uint8_t min_params[] = {to_byte(Register::min_angle_limit_l), 2};
    ok = send_command(servo_id, Instruction::read, min_params, 2);
    if (!ok) return std::nullopt;

    uint8_t min_response[8];
    ok = read_response(min_response, 8);
    if (!ok) return std::nullopt;
    info.min_angle = unpack_uint16(&min_response[5]);
    
    // Read max angle limit (registers 11-12, 2 bytes)
    uint8_t max_params[] = {to_byte(Register::max_angle_limit_l), 2};
    ok = send_command(servo_id, Instruction::read, max_params, 2);
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

    if(!send_command(servo_id, Instruction::write, torque_params, 2)) {
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

    if(!send_command(servo_id, Instruction::write, mode_params, 2)) {
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

      if(!send_command(servo_id, Instruction::write, mode_params, 2)) {
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

      if(!send_command(servo_id, Instruction::write, params, 5)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 3)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 3)) {
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

    if(!send_command(servo_id, Instruction::read, parameters, 2)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 3)) {
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

    if(!send_command(servo_id, Instruction::read, parameters, 2)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 2)) {
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

    if(!send_command(servo_id, Instruction::write, parameters, 2)) {
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

    if(!send_command(servo_id, Instruction::read, parameters, 2)) {
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
    if (!consume_echo(packet_size)) {
      return false;
    }
    
    last_error_ = ServoError::none;
    return true;
  }

  // Sync read positions from multiple servos at once
  // NOTE: SCSCL servos (SC15, SC09, etc.) may NOT support sync_read instruction
  // This is confirmed to work on SMS_STS servos
  std::vector<std::optional<int>> sync_read_positions(const std::vector<uint8_t>& servo_ids) {
    std::vector<std::optional<int>> results(servo_ids.size());
    
    if (servo_ids.empty() || servo_ids.size() > max_servo_count) {
      last_error_ = ServoError::invalid_parameter;
      return results;
    }
    
    if (servo_type_ == ServoType::SC) {
      Serial.printf("WARNING: SC servos typically don't support sync_read - attempting anyway\n");
    }
    
    // Build parameters: START_ADDR DATA_LEN ID1 ID2 ...
    uint8_t parameters[2 + max_servo_count];
    parameters[0] = to_byte(Register::present_position_l);  // Start address
    parameters[1] = 2;  // Data length (2 bytes for position)
    
    // Add servo IDs
    for (size_t i = 0; i < servo_ids.size(); i++) {
      parameters[2 + i] = servo_ids[i];
    }
    
    // Send sync_read command using existing method
    if (!send_command(to_byte(Protocol::broadcast_id), Instruction::sync_read, 
                      parameters, 2 + servo_ids.size())) {
      return results;
    }
    
    // Wait for responses from all servos
    delay(10);  // Give servos time to respond
    
    // Read all available bytes
    uint8_t response_buffer[200];
    int bytes_read = 0;
    unsigned long read_start = millis();
    
    while (bytes_read < 200 && millis() - read_start < 50) {
      if (bus_serial_->available()) {
        response_buffer[bytes_read++] = bus_serial_->read();
        read_start = millis();  // Reset timeout on each byte
      }
    }
    
    if (bytes_read == 0) {
      Serial.printf("No response from sync_read - servos may not support this command\n");
      last_error_ = ServoError::timeout;
      return results;
    }
    
    // Parse responses
    int pos = 0;
    std::map<uint8_t, int> position_map;  // Map servo_id -> position
    
    while (pos + MIN_PACKET_SIZE <= bytes_read) {
      // Look for packet header FF FF
      if (response_buffer[pos] == 0xFF && response_buffer[pos + 1] == 0xFF) {
        uint8_t servo_id = response_buffer[pos + 2];
        uint8_t length = response_buffer[pos + 3];
        uint8_t error = response_buffer[pos + 4];
        
        int packet_size = 4 + length;
        if (pos + packet_size <= bytes_read) {
          // Validate checksum
          int param_count = length - 2;  // length includes instruction + params
          uint8_t expected_checksum = calculate_checksum(
            servo_id, length, error,
            param_count > 0 ? &response_buffer[pos + 5] : nullptr,
            param_count
          );
          
          if (expected_checksum == response_buffer[pos + packet_size - 1]) {
            // Extract position (2 bytes after error byte at position 4)
            uint16_t position = unpack_uint16(&response_buffer[pos + 5]);
            position_map[servo_id] = position;
          } else {
            Serial.printf("Checksum mismatch for servo %d\n", servo_id);
          }
          pos += packet_size;
        } else {
          pos++;
        }
      } else {
        pos++;
      }
    }
    
    // Fill results array in the order requested
    for (size_t i = 0; i < servo_ids.size(); i++) {
      auto it = position_map.find(servo_ids[i]);
      if (it != position_map.end()) {
        results[i] = it->second;
      }
    }
    
    last_error_ = ServoError::none;
    return results;
  }

} servo_bus;
