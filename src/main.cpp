#include <Arduino.h>
#include <SCServo.h>

#include <optional>
#include <vector>

#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define pin_servo_tx 8
#define pin_servo_rx 18

SCSCL legacy_servo_bus;


class SCServoBus {
public:
  // Protocol byte constants
  enum class Protocol : uint8_t {
    header_byte_1 = 0xFF,
    header_byte_2 = 0xFF,
    broadcast_id = 0xFE
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
  
  // Pack 16-bit value into big-endian byte array (HIGH byte first)
  void pack_uint16_be(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;  // HIGH byte
    buffer[1] = value & 0xFF;         // LOW byte
  }
  
  // Unpack 16-bit value from big-endian byte array (HIGH byte first)
  uint16_t unpack_uint16_be(const uint8_t* buffer) {
    return (buffer[0] << 8) | buffer[1];  // HIGH | LOW
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
    
    // Extract position - SC series (potentiometer): HIGH byte first, then LOW byte
    int position = unpack_uint16_be(&response[5]);
    last_error_ = ServoError::none;
    return position;
  }

  std::optional<uint8_t> ping(uint8_t servo_id) {
    if(!send_command(servo_id, to_byte(Instruction::read))) {  // Read servo ID register
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
    // SC series (potentiometer): HIGH byte first, then LOW byte
    pack_uint16_be(&parameters[1], position);
    pack_uint16_be(&parameters[3], time_ms);
    pack_uint16_be(&parameters[5], speed);
    
    Serial.printf("write_position: position=%d (0x%02X %02X), time=%d (0x%02X %02X), speed=%d (0x%02X %02X)\n",
                  position, parameters[1], parameters[2], 
                  time_ms, parameters[3], parameters[4],
                  speed, parameters[5], parameters[6]);
    
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

void setup() {
  Serial.begin(1000000);
  delay(2000);
  Serial.println("=== Back and Forth Test ===");

  Serial1.begin(1000000, SERIAL_8N1, pin_servo_rx, pin_servo_tx);
  servo_bus.set_serial(&Serial1);

  legacy_servo_bus.pSerial = &Serial1;
  delay(500);
  scan_ids(1,100);

  // Byte order verification test - move servo #2 slowly and read position continuously
  uint8_t test_servo = 2;
  uint16_t start_position = 500;
  uint16_t end_position = 3500;
  uint16_t speed = 100;  // Very slow speed for long movement
  
  Serial.printf("\n=== Byte Order Verification Test ===\n");
  Serial.printf("Moving servo #%d from %d to %d at speed %d\n", test_servo, start_position, end_position, speed);
  Serial.printf("Expected position range: %d to %d (should increase smoothly)\n\n", start_position, end_position);
  
  // Move to start position first
  servo_bus.write_position(test_servo, start_position, 0, 500);
  delay(2000);
  
  // Start slow movement
  servo_bus.write_position(test_servo, end_position, 0, speed);
  
  // Read position continuously during movement
  unsigned long start_time = millis();
  int sample_count = 0;
  int last_position = -1;
  
  while (millis() - start_time < 35000) {  // Read for 35 seconds
    auto position = servo_bus.read_position(test_servo);
    if (position) {
      sample_count++;
      int delta = last_position >= 0 ? (*position - last_position) : 0;
      Serial.printf("Sample %3d @ %5lu ms: Position = %4d", sample_count, millis() - start_time, *position);
      if (last_position >= 0) {
        Serial.printf(" (delta: %+5d)", delta);
      }
      Serial.println();
      last_position = *position;
    } else {
      Serial.printf("Sample %3d @ %5lu ms: Read failed\n", sample_count, millis() - start_time);
    }
    delay(200);  // Sample every 200ms
  }
  
  Serial.println("\n=== Test Complete ===");
}

void loop() {}