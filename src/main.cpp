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
    
    // SRAM (read/write)
    torque_enable = 40,
    goal_position_l = 42,
    goal_position_h = 43,
    goal_time_l = 44,
    goal_time_h = 45,
    goal_speed_l = 46,
    goal_speed_h = 47,
    lock = 48,
    
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






void scan_ids(uint32_t start_id, uint32_t end_id) {
  Serial.printf("scanning for servo ids from %d to %d\n", start_id, end_id);
  for (uint32_t id = start_id; id<= end_id; ++id) {
    if (servo_bus.ping(id)) {
      Serial.printf("found servo with id %d\n",id);
    }
  }
  Serial.println("done");
}

void demonstrate_coordinated_moving() {
  Serial.println("=== Coordinated Servo Movement Test ===");

  // Servos 1, 2, 3 are SC (big-endian)
  // Servo 4 is STS (little-endian)
  
  Serial.println("\n=== Reading servo ranges ===");
  
  std::vector<uint8_t> sc_ids = {1, 2, 3};
  std::vector<uint8_t> sts_ids = {4};
  std::vector<uint16_t> sc_mins, sc_maxs, sts_mins, sts_maxs;
  
  // Read SC servo ranges
  servo_bus.set_servo_type(SCServoBus::ServoType::SC);
  Serial.println("SC servos (1, 2, 3):");
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
  Serial.println("STS servos (4):");
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

void setup() {
  Serial.begin(1000000);
  delay(2000);

  Serial1.begin(1000000, SERIAL_8N1, pin_servo_rx, pin_servo_tx);
  servo_bus.set_serial(&Serial1);
  demonstrate_coordinated_moving();
  
}

void loop() {}