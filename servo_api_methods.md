# Servo Class Hierarchy - Method Coverage

## Base Class: `Servo`

Location: [main.cpp:862-997](src/main.cpp#L862-L997)

### Public Methods

#### Getters/Info
- `uint8_t id() const` - Returns servo ID
- `uint16_t min_encoder_angle() const` - Returns minimum encoder angle
- `uint16_t max_encoder_angle() const` - Returns maximum encoder angle
- `uint16_t encoder_angle_range() const` - Returns encoder angle range
- `bool info_loaded() const` - Returns whether servo info has been loaded

#### Pure Virtual
- `virtual SCServoBus::ServoType type() const = 0` - Returns servo type (must be implemented by derived classes)

#### Position Control
- `bool read_info()` - Reads servo info (min/max angles) from servo
- `std::optional<int> read_encoder_angle()` - Reads current encoder angle
- `std::optional<int16_t> read_speed()` - Reads current speed (positive=CW, negative=CCW)
- `std::optional<int16_t> read_load()` - Reads current load/torque (positive=CW load, negative=CCW load)
- `bool move_to_encoder_angle(uint16_t encoder_angle, uint32_t duration_ms)` - Moves to encoder angle with time duration
- `bool move_to_percent(float percent, uint32_t duration_ms)` - Moves to position as percentage (0.0-1.0) of range

#### Torque Control (Available on ALL servo types)
- `bool enable_torque()` - Enables servo torque (allows motor to hold position)
- `bool disable_torque()` - Disables servo torque (motor is free to move)
- `std::optional<bool> is_torque_enabled()` - Reads torque enable status

#### PWM/Motor Mode (Available on ALL servo types)
- `bool enable_pwm_mode()` - Enables PWM open-loop speed mode
  - SC servos: Sets angle limits to 0
  - STS servos: Sets MODE register to 2
- `bool set_pwm_speed(int16_t speed)` - Sets PWM speed (positive=CW, negative=CCW, uses bit 10 for direction)

---

## Derived Class: `SCServo` (SC Series)

Location: [main.cpp:1000-1004](src/main.cpp#L1000-L1004)

### Public Methods

#### Overrides
- `SCServoBus::ServoType type() const override` - Returns `SCServoBus::ServoType::SC`

#### Inherited from Servo
- All methods from base `Servo` class

### Notes
- Uses big-endian byte order
- Only adds type specification, no additional methods beyond base class

---

## Derived Class: `STSServo` (STS Series)

Location: [main.cpp:1007-1054](src/main.cpp#L1007-L1054)

### Public Methods

#### Overrides
- `SCServoBus::ServoType type() const override` - Returns `SCServoBus::ServoType::STS`

#### Wheel Mode (STS-specific)
- `bool enable_wheel_mode()` - Enables velocity-based continuous rotation mode
- `bool set_wheel_velocity(int16_t speed)` - Sets wheel velocity (positive=CW, negative=CCW)
- `bool enable_position_mode()` - Re-enables position mode after wheel mode

#### Hardware Acceleration (STS-specific)
- `bool move_to_encoder_angle_with_accel(uint16_t encoder_angle, uint16_t speed, uint8_t acc)` - Moves to encoder angle with acceleration control
- `bool move_to_percent_with_accel(float percent, uint16_t speed, uint8_t acc)` - Moves to percentage position with acceleration control

#### Torque Limit (STS-specific)
- `bool set_torque_limit(uint16_t limit)` - Sets maximum torque output (0-1023, default 1023 = 100%)
- `uint16_t read_torque_limit()` - Reads current torque limit setting

#### Inherited from Servo
- All methods from base `Servo` class

### Notes
- Uses little-endian byte order
- Adds wheel mode (different from PWM mode)
- Adds hardware acceleration support via ACC parameter (0-255)
  - ACC=0: No acceleration
  - ACC=50: Moderate smoothing
  - ACC=200+: High smoothing

---

## Method Coverage Summary

| Method Category | Servo (Base) | SCServo | STSServo |
|----------------|--------------|---------|----------|
| Basic Info/Getters | 5 methods | ✓ Inherited | ✓ Inherited |
| Position Control | 6 methods | ✓ Inherited | ✓ Inherited |
| Torque Control | 3 methods | ✓ Inherited | ✓ Inherited |
| PWM Mode | 2 methods | ✓ Inherited | ✓ Inherited |
| Wheel Mode | - | - | ✓ 3 methods |
| Hardware Accel | - | - | ✓ 2 methods |
| Torque Limit | - | - | ✓ 2 methods |
| Type Override | 1 pure virtual | ✓ Implemented | ✓ Implemented |
| **Total Public Methods** | **17** | **17** | **24** |

---

## API Design Notes

### Inheritance Pattern
- Base class provides common functionality for all servos
- Derived classes only add hardware-specific features
- Clean separation: SC has no extras, STS has advanced features

### Mode Availability
1. **Position Mode** (All servos)
   - Basic position control via `move_to_encoder_angle()` / `move_to_percent()`
   - Available on both SC and STS

2. **PWM Mode** (All servos)
   - Open-loop speed control via `enable_pwm_mode()` / `set_pwm_speed()`
   - Implementation differs but API is unified in base class
   - Available on both SC and STS

3. **Wheel Mode** (STS only)
   - Velocity-based continuous rotation
   - Different from PWM mode - uses velocity control instead of PWM
   - Only on STS via `enable_wheel_mode()` / `set_wheel_velocity()`

4. **Hardware Acceleration** (STS only)
   - ACC parameter for smooth motion
   - Only on STS via `*_with_accel()` methods

### API Coverage Status

#### ✅ Fully Supported
All hardware-supported features are now exposed in the API:
- **Speed/velocity reading** via `read_speed()` method (both SC and STS)
- **Load/torque reading** via `read_load()` method (both SC and STS)
- **Torque enable/disable** via `enable_torque()` / `disable_torque()` (both SC and STS)
- **Torque limit control** via `set_torque_limit()` / `read_torque_limit()` (STS only)

#### Hardware Limitations
- **Acceleration control** - ACC register is STS-only (not available on SC servos)
- **Torque limit** - Only available on STS servos (SC servos lack these registers)
