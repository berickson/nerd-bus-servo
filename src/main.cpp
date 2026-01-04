#include <Arduino.h>
#include <SCServo.h>

#define pin_servo_tx 8
#define pin_servo_rx 18

SCSCL servo_bus;

int SCS2Host(uint8_t DataL, uint8_t DataH) {
  return (DataH << 8) | DataL;
}

int customReadPos(uint8_t ID) {
  while(Serial1.available()) {
    Serial1.read();
  }
  
  uint8_t packet[8];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = ID;
  packet[3] = 0x04;
  packet[4] = 0x02;
  packet[5] = 56;
  packet[6] = 2;
  packet[7] = ~(ID + 0x04 + 0x02 + 56 + 2);
  
  Serial1.write(packet, 8);
  Serial1.flush();
  
  // Discard echo
  for(int i = 0; i < 8; i++) {
    while(!Serial1.available());
    Serial1.read();
  }
  
  delay(5);
  while(Serial1.available() < 8);
  
  uint8_t response[8];
  Serial1.readBytes(response, 8);
  
  // Print raw response for debugging
  Serial.print("  Raw response: ");
  for(int i = 0; i < 8; i++) {
    Serial.print("0x");
    if(response[i] < 0x10) Serial.print("0");
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  if(response[0] != 0xFF || response[1] != 0xFF) {
    return -1;
  }
    // Extract position - response[5] is LOW byte, response[6] is HIGH byte
  int position = response[5] << 8  | (response[6]);
  return position;
  // return SCS2Host(response[5], response[6]);
}

bool custom_ping(uint8_t ID) {
  while(Serial1.available()) {
    Serial1.read();
  }
  
  uint8_t packet[6];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = ID;
  packet[3] = 0x02;
  packet[4] = 0x01;
  packet[5] = ~(ID + 0x02 + 0x01);
  
  Serial1.write(packet, 6);
  Serial1.flush();
  
  for(int i = 0; i < 6; i++) {
    unsigned long start = millis();
    while(!Serial1.available() && (millis() - start) < 50);
    if(Serial1.available()) {
      Serial1.read();
    }
  }
  
  delay(5);
  unsigned long start = millis();
  while(Serial1.available() < 6 && (millis() - start) < 100);
  
  if(Serial1.available() < 6) {
    return false;
  }
  
  uint8_t response[6];
  Serial1.readBytes(response, 6);
  
  return (response[0] == 0xFF && response[1] == 0xFF && response[2] == ID);
}


void scan_ids(uint32_t start_id, uint32_t end_id) {
  Serial.printf("scanning for servo ids from %d to %d\n", start_id, end_id);
  for (uint32_t id = start_id; id<= end_id; ++id) {
    if (custom_ping(id)) {
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
  servo_bus.pSerial = &Serial1;
  delay(500);
  scan_ids(1,10);

  uint32_t servo_id = 3;
  
  for(int i = 0; i < 3; i++) {
    Serial.println("\n--- Moving to 24 ---");
    servo_bus.WritePos(servo_id, 24, 0, 1500);
    delay(2000);
    
    int pos = customReadPos(servo_id);
    Serial.print("Position: ");
    Serial.println(pos);
    delay(1000);
    
    Serial.println("\n--- Moving to 500 ---");
    servo_bus.WritePos(servo_id, 500, 0, 1500);
    delay(2000);
    
    pos = customReadPos(servo_id);
    Serial.print("Position: ");
    Serial.println(pos);
    delay(1000);
  }
  
  Serial.println("\nDone!");
}

void loop() {}