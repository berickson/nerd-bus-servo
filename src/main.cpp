#include <Arduino.h>
#include <SCServo.h>

#include <vector>

#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define pin_servo_tx 8
#define pin_servo_rx 18

SCSCL legacy_servo_bus;

class SCServoBus {
public:

  bool send_command(uint8_t id, uint8_t instruction, uint8_t* params = nullptr, int param_count = 0) {

    // clear the rx
    if (Serial1.available()) {
      Serial.println("Clearing extra rx before sending a command");
      while (Serial1.available()) {
        auto b = Serial1.read();
        Serial.print(b, 16);
      }
      Serial.println();
    }

    uint8_t packet[256];  // Max packet size
    
    // Build packet
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 2 + param_count;  // LENGTH = instruction byte + params
    packet[4] = instruction;
    
    // Copy parameters
    for(int i = 0; i < param_count; i++) {
      packet[5 + i] = params[i];
    }
    
    // Calculate checksum: ~(ID + LENGTH + INSTRUCTION + PARAMETERS)
    uint8_t checksum = id + packet[3] + instruction;
    for(int i = 0; i < param_count; i++) {
      checksum += params[i];
    }
    packet[5 + param_count] = ~checksum;
    
    int packet_size = 6 + param_count;
    
    // Send packet
    Serial1.write(packet, packet_size);
    
    // Discard echo bytes as they arrive during transmission
    int echo_count = 0;
    while(echo_count < packet_size) {
      if(Serial1.available()) {
        Serial1.read();
        echo_count++;
      }
    }

    return true;
  }

  bool read_response(uint8_t* response, int expected_size, int timeout_ms = 100) {
    unsigned long start = millis();
    
    // Wait for expected response size
    while(Serial1.available() < expected_size && (millis() - start) < timeout_ms);
    
    if(Serial1.available() < expected_size) {
      return false;  // Timeout
    }
    
    Serial1.readBytes(response, expected_size);
    
    // Validate header
    if(response[0] != 0xFF || response[1] != 0xFF) {
      return false;
    }
    
    // Validate checksum: ~(ID + LENGTH + INSTRUCTION + PARAMETERS)
    uint8_t checksum = response[2];  // ID
    checksum += response[3];  // LENGTH
    for(int i = 4; i < expected_size - 1; i++) {
      checksum += response[i];
    }
    checksum = ~checksum;
    
    if(checksum != response[expected_size - 1]) {
      return false;  // Checksum mismatch
    }
    
    return true;
  }

  int read_position(uint8_t servo_id) {
    uint8_t params[] = {56, 2};  // Address 56, read 2 bytes
    send_command(servo_id, 0x02, params, 2);  // 0x02 = READ instruction
    
    uint8_t response[8];
    if(!read_response(response, 8)) {
      return -1;
    }
    
    
    // Extract position - response[5] is LOW byte, response[6] is HIGH byte
    int position = response[5] << 8  | (response[6]);
    return position;
  }

  bool ping(uint8_t ID) {
    send_command(ID, 0x01);  // 0x01 = PING instruction, no params
    
    uint8_t response[6];
    if(!read_response(response, 6)) {
      return false;
    }
    
    return (response[2] == ID);  // Header already validated in custom_read_response
  }

  bool write_pos(uint8_t servo_id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    legacy_servo_bus.WritePos(servo_id, position, time_ms, speed);
    return true;
    // Write 6 bytes to address 42: position(2), time(2), speed(2)
    uint8_t params[7];
    params[0] = 42;  // Start address (SCSCL_GOAL_POSITION_L)
    // Host2SCS writes LOW byte first, then HIGH byte (little endian)
    params[1] = (position >> 8) & 0xFF;       // Position high byte
    params[2] = (position) & 0xFF;            // Position low byte
    params[3] = (time_ms >> 8) & 0xFF;        // Time high byte
    params[4] = (time_ms) & 0xFF;             // Time low byte
    params[5] = (speed >> 8) & 0xFF;          // Speed high byte
    params[6] = (speed) & 0xFF;               // Speed low byte
    
    Serial.printf("WritePos: ID=%d, Pos=%d, Time=%d, Speed=%d\n", servo_id, position, time_ms, speed);
    Serial.printf("  Params: addr=42, pos=0x%02X%02X, time=0x%02X%02X, speed=0x%02X%02X\n",
      params[2], params[1], params[4], params[3], params[6], params[5]);
    
    if(!send_command(servo_id, 0x03, params, 7)) {  // 0x03 = WRITE instruction, 7 bytes total (addr + 6 data bytes)
      return false;
    }
    
    // Read ACK response
    uint8_t response[6];
    if(!read_response(response, 6)) {
      return false;
    }
    
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
  scan_ids(1,10);


  std::vector<uint32_t> servo_ids = {1,2,3};
  std::vector<uint32_t> setpoints = {50,200};

  for (auto servo_id : servo_ids ) {
    for (auto setpoint : setpoints) {

      int start_position = servo_bus.read_position(servo_id);
      Serial.printf("Moving servo_id %d from current position of %d to %d\n", servo_id, start_position, setpoint);
      servo_bus.write_pos(servo_id, setpoint, 0, 300);
      while(Serial1.available()) {
        Serial1.read();
      }
      delay(2000);
    }
    int final_position = servo_bus.read_position(servo_id);
    Serial.printf("servo_id %d now at %d\n", servo_id, final_position);
  }
  
  Serial.println("\nDone!");
}

void loop() {}