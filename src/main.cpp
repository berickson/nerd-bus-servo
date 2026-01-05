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
  // Protocol constants
  static constexpr uint8_t PACKET_HEADER_BYTE_1 = 0xFF;
  static constexpr uint8_t PACKET_HEADER_BYTE_2 = 0xFF;
  static constexpr uint8_t BROADCAST_ID = 0xFE;
  static constexpr int HEADER_SIZE = 2;
  static constexpr int MIN_PACKET_SIZE = 6;  // header(2) + id(1) + length(1) + instruction(1) + checksum(1)
  static constexpr int INSTRUCTION_OVERHEAD = 2;  // instruction byte + params (length field = instruction + params)
  
  // Packet field offsets
  static constexpr int OFFSET_HEADER1 = 0;
  static constexpr int OFFSET_HEADER2 = 1;
  static constexpr int OFFSET_ID = 2;
  static constexpr int OFFSET_LENGTH = 3;
  static constexpr int OFFSET_INSTRUCTION = 4;
  static constexpr int OFFSET_PARAMS = 5;

  // Servo memory register addresses (from SCSCL.h)
  enum class Register : uint8_t {
    // EPROM (read-only)
    VersionL = 3,
    VersionH = 4,
    
    // EPROM (read/write)
    Id = 5,
    BaudRate = 6,
    MinAngleLimitL = 9,
    MinAngleLimitH = 10,
    MaxAngleLimitL = 11,
    MaxAngleLimitH = 12,
    CwDead = 26,
    CcwDead = 27,
    
    // SRAM (read/write)
    TorqueEnable = 40,
    GoalPositionL = 42,
    GoalPositionH = 43,
    GoalTimeL = 44,
    GoalTimeH = 45,
    GoalSpeedL = 46,
    GoalSpeedH = 47,
    Lock = 48,
    
    // SRAM (read-only)
    PresentPositionL = 56,
    PresentPositionH = 57,
    PresentSpeedL = 58,
    PresentSpeedH = 59,
    PresentLoadL = 60,
    PresentLoadH = 61,
    PresentVoltage = 62,
    PresentTemperature = 63,
    Moving = 66,
    PresentCurrentL = 69,
    PresentCurrentH = 70
  };

  // Protocol instruction codes (from INST.h)
  enum class Instruction : uint8_t {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    RegAction = 0x05,
    SyncRead = 0x82,
    SyncWrite = 0x83
  };

  enum class ServoError {
    None = 0,
    Timeout,
    InvalidHeader,
    ChecksumMismatch,
    InvalidResponse,
    InvalidParameter,
    NoAck
  };


  private:
  ServoError last_error_ = ServoError::None;
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
                                      const uint8_t* params = nullptr, int param_count = 0) {
    uint8_t sum = id + length + instruction;
    for(int i = 0; i < param_count; i++) {
      sum += params[i];
    }
    return ~sum;
  }

  // Serial configuration
  void set_serial(HardwareSerial* serial) { bus_serial_ = serial; }
  
  // Error state accessors
  inline bool ok() const { return last_error_ == ServoError::None; }
  inline ServoError last_error() const { return last_error_; }
  inline void clear_error() { last_error_ = ServoError::None; }

  bool send_command(uint8_t id, uint8_t instruction, uint8_t* params = nullptr, int param_count = 0) {
    const int max_param_count = max_servo_count + 2;
    if (param_count > max_param_count) {
      last_error_ = ServoError::InvalidParameter;
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
    packet[OFFSET_HEADER1] = PACKET_HEADER_BYTE_1;
    packet[OFFSET_HEADER2] = PACKET_HEADER_BYTE_2;
    packet[OFFSET_ID] = id;
    packet[OFFSET_LENGTH] = INSTRUCTION_OVERHEAD + param_count;  // LENGTH = instruction byte + params
    packet[OFFSET_INSTRUCTION] = instruction;
    
    // Copy parameters
    for(int i = 0; i < param_count; i++) {
      packet[OFFSET_PARAMS + i] = params[i];
    }
    
    // Calculate and append checksum
    packet[OFFSET_PARAMS + param_count] = calculate_checksum(id, packet[OFFSET_LENGTH], instruction, params, param_count);
    
    int packet_size = MIN_PACKET_SIZE + param_count;
    
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
        last_error_ = ServoError::Timeout;
        return false;
      }
    }

    last_error_ = ServoError::None;
    return true;
  }

  bool read_response(uint8_t* response, int expected_size) {
    unsigned long start_ms = millis();;
    
    // Wait for expected response size
    while(bus_serial_->available() < expected_size && millis()-start_ms < timeout_ms_);
    
    if(bus_serial_->available() < expected_size) {
      last_error_ = ServoError::Timeout;
      return false;
    }
    
    bus_serial_->readBytes(response, expected_size);
    
    // Validate header
    if(response[OFFSET_HEADER1] != PACKET_HEADER_BYTE_1 || response[OFFSET_HEADER2] != PACKET_HEADER_BYTE_2) {
      last_error_ = ServoError::InvalidHeader;
      return false;
    }
    
    // Validate checksum
    int param_count = expected_size - MIN_PACKET_SIZE;
    uint8_t expected_checksum = calculate_checksum(
      response[OFFSET_ID], 
      response[OFFSET_LENGTH], 
      response[OFFSET_INSTRUCTION],
      param_count > 0 ? &response[OFFSET_PARAMS] : nullptr,
      param_count
    );
    
    if(expected_checksum != response[expected_size - 1]) {
      last_error_ = ServoError::ChecksumMismatch;
      return false;
    }
    
    last_error_ = ServoError::None;
    return true;
  }

  std::optional<int> read_position(uint8_t servo_id) {
    uint8_t params[] = {static_cast<uint8_t>(Register::PresentPositionL), 2};  // Read current position (2 bytes)
    if(!send_command(servo_id, static_cast<uint8_t>(Instruction::Read), params, 2)) {
      return std::nullopt;
    }
    
    uint8_t response[8];
    if(!read_response(response, 8)) {
      return std::nullopt;
    }
    
    // Extract position - SC series (potentiometer): HIGH byte first, then LOW byte
    int position = unpack_uint16_be(&response[5]);
    last_error_ = ServoError::None;
    return position;
  }

  std::optional<uint8_t> ping(uint8_t ID) {
    if(!send_command(ID, static_cast<uint8_t>(Instruction::Read))) {  // Read servo ID register
      return std::nullopt;
    }
    
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      return std::nullopt;
    }
    
    if(response[OFFSET_ID] != ID) {
      last_error_ = ServoError::InvalidResponse;
      return std::nullopt;
    }
    
    last_error_ = ServoError::None;
    return response[OFFSET_ID];
  }

  bool write_pos(uint8_t servo_id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    // legacy_servo_bus.WritePos(servo_id, position, time_ms, speed);
    // last_error_ = ServoError::None;
    // return true;
    // Write 6 bytes starting at GoalPositionL: position(2), time(2), speed(2)
    uint8_t params[7];
    params[0] = static_cast<uint8_t>(Register::GoalPositionL);
    // SC series (potentiometer): HIGH byte first, then LOW byte
    pack_uint16_be(&params[1], position);
    pack_uint16_be(&params[3], time_ms);
    pack_uint16_be(&params[5], speed);
    
    Serial.printf("write_pos: pos=%d (0x%02X %02X), time=%d (0x%02X %02X), speed=%d (0x%02X %02X)\n",
                  position, params[1], params[2], 
                  time_ms, params[3], params[4],
                  speed, params[5], params[6]);
    
    if(!send_command(servo_id, static_cast<uint8_t>(Instruction::Write), params, 7)) {
      return false;
    }
    
    // Read ACK response
    uint8_t response[MIN_PACKET_SIZE];
    if(!read_response(response, MIN_PACKET_SIZE)) {
      last_error_ = ServoError::NoAck;
      return false;
    }
    
    last_error_ = ServoError::None;
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

  // gpio_config_t io_conf = {};
  // io_conf.pin_bit_mask = (1ULL << pin_servo_tx);
  // io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  // io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  // gpio_config(&io_conf);

  // // Reconnect UART after GPIO config
  // esp_rom_gpio_connect_out_signal(pin_servo_tx, U1TXD_OUT_IDX, false, false);
  // esp_rom_gpio_connect_in_signal(pin_servo_rx, U1RXD_IN_IDX, false);


  legacy_servo_bus.pSerial = &Serial1;
  delay(500);
  scan_ids(1,100);

  // Byte order verification test - move servo #2 slowly and read position continuously
  uint8_t test_servo = 2;
  uint16_t start_pos = 500;
  uint16_t end_pos = 3500;
  uint16_t speed = 100;  // Very slow speed for long movement
  
  Serial.printf("\n=== Byte Order Verification Test ===\n");
  Serial.printf("Moving servo #%d from %d to %d at speed %d\n", test_servo, start_pos, end_pos, speed);
  Serial.printf("Expected position range: %d to %d (should increase smoothly)\n\n", start_pos, end_pos);
  
  // Move to start position first
  servo_bus.write_pos(test_servo, start_pos, 0, 500);
  delay(2000);
  
  // Start slow movement
  servo_bus.write_pos(test_servo, end_pos, 0, speed);
  
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