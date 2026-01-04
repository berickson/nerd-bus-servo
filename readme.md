# ESP32 Direct SC Servo Control

This project demonstrates how to control Feetech/Waveshare SC series servos (SC09, SC15, etc.) directly from an ESP32 microcontroller **without using Waveshare servo driver boards**.

## Reference Documents

For general communication:
https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf

For specific commands and addresses:
https://engineering.purdue.edu/477grp4/Team/journal/img%20-%20Juho/week7/servo%20bus%20protocol.pdf

## Solution Overview

**Hardware**: Two GPIO pins + one resistor
- GPIO 8 (TX) → 1kΩ resistor → Servo data line
- GPIO 18 (RX) → Servo data line (direct)
- 10kΩ pull-up resistor to 3.3V on data line

**Software**: Standard SCServo library for writes, custom implementations for reads
- Baud rate: 1000000 (1 Mbps)
- Library write functions work as-is
- Read functions require custom implementations to handle TX echo

## Why This Approach?

The Waveshare servo driver boards use tri-state buffers with direction control (TXEN signal) to separate TX and RX. Our solution achieves the same result with:
- **1kΩ resistor on TX**: Limits drive strength, allows servo to override when responding
- **Direct RX connection**: Reads everything on the bus (both echo and servo responses)
- **Software echo handling**: Custom read functions discard echo bytes before reading servo data

## Critical Implementation Details

### The Echo Problem

When TX and RX share the same data line, the ESP32 receives everything it transmits (echo). This causes:
1. Library read functions (Ping, ReadPos, etc.) to read echo instead of servo responses
2. All read operations to return garbage data

### The Solution

**Write operations** (WritePos, EnableTorque, etc.):
- Use standard SCServo library functions
- Work correctly because they wait for ACK responses
- The 1kΩ resistor allows servo to send ACK

**Read operations** (Ping, ReadPos, ReadSpeed, etc.):
- Require custom implementations
- Must discard echo bytes before reading servo response
- Pattern: Send command → Discard N echo bytes → Read servo response

### Echo Handling Pattern

1. Flush RX buffer
2. Send command packet (N bytes)
3. **Discard exactly N bytes** (the echo)
4. Wait for servo response
5. Read and parse servo response

The number of echo bytes always equals the number of bytes sent.

## What Works vs What Doesn't

### ✅ Works with Library (No Modification)
- `WritePos()` - Move to position
- `WritePosEx()` - Move with acceleration
- `EnableTorque()` - Enable/disable holding torque
- `writeByte()` / `writeWord()` - Write to servo memory
- Any write operation that expects ACK

### ❌ Requires Custom Implementation
- `Ping()` - Check servo presence
- `ReadPos()` - Read current position
- `ReadSpeed()` - Read current speed
- `ReadLoad()` - Read current load
- `ReadVoltage()` - Read supply voltage
- `ReadTemper()` - Read temperature
- Any read operation expecting data response

## Key Differences from Waveshare Boards

**Waveshare Approach**:
- Hardware tri-state buffers (SN74LVC1G126DBVR)
- TXEN signal for direction control
- Clean separation between TX and RX

**Our Approach**:
- 1kΩ resistor to limit TX drive
- Software discards echo bytes
- Simpler hardware, requires custom read code

## Protocol Details

### SC Servo Packet Structure
```
[0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMETERS...] [CHECKSUM]
```

- Header: Always 0xFF 0xFF
- ID: Servo ID (1-253, 254=broadcast)
- Checksum: ~(ID + LENGTH + INSTRUCTION + PARAMETERS)

### Key Memory Addresses (SCSCL Series)
- SCSCL_GOAL_POSITION_L: 42 (write target position)
- SCSCL_PRESENT_POSITION_L: 56 (read current position)

Positions are 16-bit values, typical range 0-4095.

## Byte Order Note

Position data in responses is **HIGH byte first, LOW byte second**:
```
Position = (response[5] << 8) | response[6]
```

## Hardware Setup

1. **Power**: Servos need 6-8.4V external supply (not from ESP32)
2. **Grounds**: ESP32 GND, servo power GND, and servo signal GND must be tied together
3. **Data line**: 10kΩ pull-up to 3.3V recommended (internal pull-up may work but external is more reliable)
4. **Multiple servos**: Can daisy-chain on same bus, each needs unique ID

## What We Tried That Didn't Work

- **RS485 half-duplex mode** (`UART_MODE_RS485_HALF_DUPLEX`): No servo responses received
- **Open-drain mode alone**: Echo problem persisted, responses unreliable
- **TX-only (no RX)**: Write operations need ACK, so RX is required
- **Single GPIO for TX/RX without resistor**: Bus contention prevented servo responses

## Libraries

- **PlatformIO**: `workloads/SCServo @ ^1.0.1`
- Works with SCSCL, SMS_STS servo series
- Library handles packet formatting and checksums correctly
- Only read functions need custom implementations

## Code Structure

See `SC_Servo_ESP32_Direct_Control.md` for complete code examples including:
- Custom Ping implementation
- Custom ReadPos implementation
- Complete working example
- Pattern for implementing other read functions

## Board Design Recommendations

For custom PCBs:
- 2 GPIOs per servo bus (TX + RX)
- 1kΩ series resistor on TX line
- 10kΩ pull-up on data line
- Bulk capacitance (100μF+) near servo connectors
- Separate power plane for servo supply

## Troubleshooting

**Servo doesn't move**: Check power, grounds tied together, torque enabled
**ReadPos returns garbage**: Verify echo discarding, check byte order, confirm 1kΩ resistor present
**Intermittent communication**: Verify baud rate (1000000), check resistor values, ensure grounds connected
**No response from servo**: Add external pull-up, verify servo has power, check data line connection

## Development Notes for AI Assistants

When working with this codebase:
1. **Write operations**: Use library functions as-is
2. **Read operations**: Always implement custom functions with echo handling
3. **Echo bytes**: Count matches sent bytes exactly
4. **Byte order**: Position data is HIGH byte then LOW byte
5. **Timing**: 5ms delay after discarding echo helps reliability
6. **Never** assume library read functions work - they read echo instead of servo data

The 1kΩ resistor on TX is **critical** - without it, the ESP32 TX fights the servo's response and communication fails.