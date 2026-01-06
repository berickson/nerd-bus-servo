# Complete Register Inventory

## Purpose

This is the **master register inventory** that cross-references all register sources:
- SCSCL.h (SC servo library header)
- SMS_STS.h (STS servo library header)
- main.cpp (your custom implementation)
- Manufacturer documentation (where available)

## Register Map

| Addr | SC Name (SCSCL.h) | STS Name (SMS_STS.h) | In main.cpp Enum? | SC Used? | STS Used? | Discrepancy Type | Severity | Notes |
|------|-------------------|----------------------|-------------------|----------|-----------|------------------|----------|-------|
| 0 | - | - | ✗ | - | - | None | - | Undefined |
| 1 | - | - | ✗ | - | - | None | - | Undefined |
| 2 | - | - | ✗ | - | - | None | - | Undefined |
| **3** | VERSION_L | MODEL_L | ✓ (version_l) | ✓ | ✓ | None | ✅ OK | EPROM-RO, firmware version |
| **4** | VERSION_H | MODEL_H | ✓ (version_h) | ✓ | ✓ | None | ✅ OK | EPROM-RO, firmware version |
| **5** | ID | ID | ✓ (id) | ✓ | ✓ | None | ✅ OK | EPROM-RW, servo ID (0-253) |
| **6** | BAUD_RATE | BAUD_RATE | ✓ (baud_rate) | ✓ | ✓ | None | ✅ OK | EPROM-RW, communication baud |
| 7 | - | - | ✗ | - | - | None | - | Undefined |
| 8 | - | - | ✗ | - | - | None | - | Undefined |
| **9** | MIN_ANGLE_LIMIT_L | MIN_ANGLE_LIMIT_L | ✓ (min_angle_limit_l) | ✓ | ✓ | None | ✅ OK | EPROM-RW, lower limit |
| **10** | MIN_ANGLE_LIMIT_H | MIN_ANGLE_LIMIT_H | ✓ (min_angle_limit_h) | ✓ | ✓ | None | ✅ OK | EPROM-RW, lower limit |
| **11** | MAX_ANGLE_LIMIT_L | MAX_ANGLE_LIMIT_L | ✓ (max_angle_limit_l) | ✓ | ✓ | None | ✅ OK | EPROM-RW, upper limit |
| **12** | MAX_ANGLE_LIMIT_H | MAX_ANGLE_LIMIT_H | ✓ (max_angle_limit_h) | ✓ | ✓ | None | ✅ OK | EPROM-RW, upper limit |
| 13-25 | - | - | ✗ | - | - | None | - | Undefined |
| **26** | CW_DEAD | CW_DEAD | ✓ (cw_dead) | ? | ? | None | ✅ OK | EPROM-RW, CW deadband |
| **27** | CCW_DEAD | CCW_DEAD | ✓ (ccw_dead) | ? | ? | None | ✅ OK | EPROM-RW, CCW deadband |
| 28-30 | - | - | ✗ | - | - | None | - | Undefined |
| **31** | N/A | OFS_L | ✓ (ofs_l) | N/A | ✓ | None | ✅ OK | STS ONLY - Offset calibration L, now implemented |
| **32** | N/A | OFS_H | ✓ (ofs_h) | N/A | ✓ | None | ✅ OK | STS ONLY - Offset calibration H, now implemented |
| **33** | **N/A** | **MODE** | **✓ (mode)** | **⚠️** | **✓** | **Type B** | **🚨 CRITICAL** | **BUG: SC servos don't have MODE register!** |
| 34-39 | - | - | ✗ | - | - | None | - | Undefined |
| **40** | TORQUE_ENABLE | TORQUE_ENABLE | ✓ (torque_enable) | ✓ | ✓ | None | ✅ OK | SRAM-RW, enable/disable torque |
| **41** | N/A | ACC | ✓ (acc) | N/A | ✓ | None | ✅ OK | STS ONLY - Acceleration control, correctly guarded |
| **42** | GOAL_POSITION_L | GOAL_POSITION_L | ✓ (goal_position_l) | ✓ | ✓ | None | ✅ OK | SRAM-RW, target position |
| **43** | GOAL_POSITION_H | GOAL_POSITION_H | ✓ (goal_position_h) | ✓ | ✓ | None | ✅ OK | SRAM-RW, target position |
| **44** | GOAL_TIME_L | GOAL_TIME_L | ✓ (goal_time_l) | ✓ | ✓ | None | ✅ OK | SRAM-RW, movement time or PWM |
| **45** | GOAL_TIME_H | GOAL_TIME_H | ✓ (goal_time_h) | ✓ | ✓ | None | ✅ OK | SRAM-RW, movement time or PWM |
| **46** | GOAL_SPEED_L | GOAL_SPEED_L | ✓ (goal_speed_l) | ✓ | ✓ | None | ✅ OK | SRAM-RW, movement speed |
| **47** | GOAL_SPEED_H | GOAL_SPEED_H | ✓ (goal_speed_h) | ✓ | ✓ | None | ✅ OK | SRAM-RW, movement speed |
| **48** | **LOCK** | **TORQUE_LIMIT_L** | **✓ (lock_sc, torque_limit_l)** | **✓** | **✓** | **Type D** | ✅ **OK** | **Conflict handled with runtime validation** |
| **49** | N/A | TORQUE_LIMIT_H | ✓ (torque_limit_h) | N/A | ✓ | None | ✅ OK | STS ONLY - Torque limit H, now implemented |
| 50-54 | - | - | ✗ | - | - | None | - | Undefined |
| **55** | N/A | LOCK | ✓ (lock_sts) | N/A | ✓ | None | ✅ OK | STS ONLY - EEPROM lock |
| **56** | PRESENT_POSITION_L | PRESENT_POSITION_L | ✓ (present_position_l) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current position |
| **57** | PRESENT_POSITION_H | PRESENT_POSITION_H | ✓ (present_position_h) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current position |
| **58** | PRESENT_SPEED_L | PRESENT_SPEED_L | ✓ (present_speed_l) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current speed |
| **59** | PRESENT_SPEED_H | PRESENT_SPEED_H | ✓ (present_speed_h) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current speed |
| **60** | PRESENT_LOAD_L | PRESENT_LOAD_L | ✓ (present_load_l) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current load |
| **61** | PRESENT_LOAD_H | PRESENT_LOAD_H | ✓ (present_load_h) | ✓ | ✓ | None | ✅ OK | SRAM-RO, current load |
| **62** | PRESENT_VOLTAGE | PRESENT_VOLTAGE | ✓ (present_voltage) | ✓ | ✓ | None | ✅ OK | SRAM-RO, supply voltage |
| **63** | PRESENT_TEMPERATURE | PRESENT_TEMPERATURE | ✓ (present_temperature) | ✓ | ✓ | None | ✅ OK | SRAM-RO, temperature |
| 64-65 | - | - | ✗ | - | - | None | - | Undefined |
| **66** | MOVING | MOVING | ✓ (moving) | ✓ | ✓ | None | ✅ OK | SRAM-RO, movement status flag |
| 67-68 | - | - | ✗ | - | - | None | - | Undefined |
| **69** | PRESENT_CURRENT_L | PRESENT_CURRENT_L | ✓ (present_current_l) | ? | ? | None | ✅ OK | SRAM-RO, motor current |
| **70** | PRESENT_CURRENT_H | PRESENT_CURRENT_H | ✓ (present_current_h) | ? | ? | None | ✅ OK | SRAM-RO, motor current |
| 71-255 | - | - | ✗ | - | - | None | - | Undefined |

## Summary Statistics

### Register Counts
- **Total defined in SCSCL.h**: 23 registers
- **Total defined in SMS_STS.h**: 27 registers
- **Common registers**: 21 registers
- **SC-only**: 2 registers (VERSION naming, LOCK at 48)
- **STS-only**: 6 registers (OFS_L/H, MODE, ACC, TORQUE_LIMIT_L/H, LOCK at 55)

### Implementation Status
- **In main.cpp enum**: 28 registers (added 4 new STS registers)
- **Fully implemented**: 21 common registers ✅
- **SC-specific implemented**: LOCK at 48 ✅
- **STS-specific implemented**: MODE ✅, ACC ✅, LOCK at 55 ✅, OFS_L/H ✅, TORQUE_LIMIT_L/H ✅

### Discrepancies Found
| Type | Count | Severity | Description | Status |
|------|-------|----------|-------------|---------|
| **Type A** | ~~3~~ 0 | ~~MEDIUM~~ | ~~Registers in library but missing from enum~~ | ✅ **ALL IMPLEMENTED** |
| **Type B** | 1 | ~~CRITICAL~~ | ~~MODE register used on SC servos~~ | ✅ **FIXED** |
| **Type D** | 1 | ~~HIGH~~ | ~~Address 48 conflict (LOCK vs TORQUE_LIMIT_L)~~ | ✅ **RESOLVED with runtime validation** |

## Critical Issues

### ✅ Register 33 - MODE (FIXED)

**Problem**: Code was using `mode = 33` for both SC and STS servos without type checking.

**Evidence**:
- ❌ SCSCL.h: MODE register **NOT DEFINED**
- ✅ SMS_STS.h: `#define SMS_STS_MODE 33`
- ✅ main.cpp: **NOW FIXED** - Checks servo type before MODE register access

**Fix Applied**:
- `SCServoBus::enable_wheel_mode()` [line 483](../src/main.cpp#L483): Now checks `servo_type_ != ServoType::STS` and rejects SC servos
- `SCServoBus::restore_position_mode()` [line 526](../src/main.cpp#L526): Now uses MODE for STS, angle limits for SC
- SC servos use angle limits (0,0) for PWM mode, NOT MODE register
- STS servos use MODE register (0=position, 1=wheel, 2=PWM)

**Status**: ✅ RESOLVED

### ✅ Register 48 - Address Conflict (RESOLVED)

**Problem**: Same address has different meaning per servo type.

**Evidence**:
- SC (SCSCL.h): `#define SCSCL_LOCK 48`
- STS (SMS_STS.h): `#define SMS_STS_TORQUE_LIMIT_L 48`
- main.cpp: Uses both `lock_sc = 48` and `torque_limit_l = 48`

**Solution Implemented**:
- Both registers added to enum with clear comments
- Runtime validation in `is_register_valid_for_type()` prevents cross-usage
- `write_byte()` validates register access before writing
- Clear error messages guide users if they try to use wrong register

**Status**: ✅ FULLY RESOLVED with runtime validation

### ✅ Missing STS Registers (ALL IMPLEMENTED)

**Registers now in enum and implemented**:
1. ✅ OFS_L (31) - Offset calibration low byte - [main.cpp:58](../src/main.cpp#L58)
2. ✅ OFS_H (32) - Offset calibration high byte - [main.cpp:59](../src/main.cpp#L59)
3. ✅ TORQUE_LIMIT_L (48) - Torque limit low byte - [main.cpp:72](../src/main.cpp#L72)
4. ✅ TORQUE_LIMIT_H (49) - Torque limit high byte - [main.cpp:73](../src/main.cpp#L73)

**Helper Functions**:
- `set_offset()` / `read_offset()` - [main.cpp:643-692](../src/main.cpp#L643-L692)
- `set_torque_limit()` / `read_torque_limit()` - [main.cpp:696-751](../src/main.cpp#L696-L751)

**Status**: ✅ ALL FEATURES NOW AVAILABLE

## Byte Order Reference

| Servo Type | Byte Order | Functions to Use |
|------------|------------|------------------|
| SC (SCSCL) | **Big-Endian** | `pack_uint16_be()`, `unpack_uint16_be()` |
| STS (SMS_STS) | **Little-Endian** | `pack_uint16_le()`, `unpack_uint16_le()` |

**Example**: Value 0x1234
- SC writes: `[0x12, 0x34]` (HIGH, LOW)
- STS writes: `[0x34, 0x12]` (LOW, HIGH)

## Next Steps

1. ✅ Register inventory complete
2. 🔴 Document MODE register discrepancy in detail
3. 🔴 Fix MODE register usage in main.cpp
4. 🔴 Add runtime validation for servo-specific registers
5. ⚠️ Consider adding missing STS registers (OFS, TORQUE_LIMIT_H)
6. ✅ Create test suite to validate register usage

## Legend

- ✅ **OK**: Correctly implemented
- ⚠️ **MEDIUM**: Missing feature, not critical
- **HIGH**: Potential for register confusion
- 🚨 **CRITICAL**: Active bug, needs immediate fix
- **Type A**: In library, not in implementation
- **Type B**: In implementation, shouldn't be (or wrong type)
- **Type D**: Address conflict between servo types
