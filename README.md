# nerd-bus-servo  
ESP32 Direct Bus Servo Control

This library provides software drivers to control Feetech/Waveshare SC and STS series servos (SC09, SC15,STS3215 etc.) directly from an ESP32 microcontroller **without using Waveshare servo driver boards**. Different families of servo can operate on the same bus at the same time.

## Hardware Setup

Tested on ESP32 S3 using HardwareSerial with this simple circuit:
- GPIO 8 (TX) → **1kΩ resistor** → Servo data line
- GPIO 18 (RX) → Servo data line (direct connection)

**Why the resistor?** The 1kΩ resistor limits TX drive strength, allowing the servo to drive the line when responding. Without it, TX and servo responses fight each other and communication fails. The RX line reads all bus traffic (both echo and servo responses), which the software filters appropriately.

Note: This is still experimental, so use at your own risk

## Library Structure

- `servo_bus_api.h` - handles calls to the servo bus
- `servo.h` - base class for Servo wrapper classes
- `sts_servo.h` - STSServo wrapper class for STS style servos
- `sc_servo.h` - SCServo wrapper class for SC servos

In general, you should prefer using the wrapper classes over the ServoBusApi

The library uses `std::optional` for error-safe return values and internal timeouts (default 2ms) to handle communication failures robustly without blocking indefinitely.

## How It Works

Since TX and RX share the same data line, the ESP32 receives everything it transmits (echo). The library implements pseudo half-duplex by:
1. Flushing RX buffer
2. Sending command packet (N bytes)
3. Discarding exactly N echo bytes
4. Reading servo response



## Reference Documents

**General Communication Protocol:**
- https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf

**STS Servo Documentation:**
- [ST3215 Servo User Manual (PDF)](https://download.kamami.pl/p1181056-ST3215_Servo_User_Manual.pdf)
- [ST3215 Servo - Waveshare Wiki](https://www.waveshare.com/wiki/ST3215_Servo)
- [ST3215 Series Product Page](https://www.waveshare.com/st3215-servo.htm)