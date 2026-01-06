# Servo Register Validation & Completeness Tracking Plan

## Problem Statement

Recent bugs involving:
- Using wrong registers for operations
- Using right registers for wrong servo type (SC vs STS)
- Using registers in the wrong way (byte order, value encoding)

## Solution: Systematic Validation & Tracking System

This plan establishes a **methodical, repeatable process** for identifying and documenting all discrepancies between:
1. Official library headers (SCSCL.h, SMS_STS.h)
2. Official library implementation (SCSCL.cpp, SMS_STS.cpp)
3. Your custom implementation (main.cpp)
4. External documentation (Waveshare PDFs, datasheets)

We will use the MODE register discrepancy as a **working template** for how to document, verify, and fix all such issues.

---

## Systematic Methodology

### Phase 1: Register Inventory & Cross-Reference

**Goal**: Build complete register inventory from all sources

**Sources to Cross-Reference**:
1. **SCSCL.h** - Official SC servo library header (23 registers)
2. **SMS_STS.h** - Official STS servo library header (27 registers)
3. **SCSCL.cpp** - SC servo implementation patterns
4. **SMS_STS.cpp** - STS servo implementation patterns
5. **main.cpp** - Your custom implementation enum
6. **Waveshare Communication Protocol Manual** (PDF) - Manufacturer protocol spec
7. **ST3215 User Manual** (PDF) - STS servo datasheet
8. **Waveshare ST3215 Wiki** - Online reference

**Process**:
1. Extract all register definitions from SCSCL.h → SC register list
2. Extract all register definitions from SMS_STS.h → STS register list
3. Extract all register usage from SCSCL.cpp → SC implementation behaviors
4. Extract all register usage from SMS_STS.cpp → STS implementation behaviors
5. Extract all registers from your enum in main.cpp → Current implementation
6. **Fetch and parse Waveshare Communication Protocol PDF** → Official protocol register map
7. **Fetch and parse ST3215 User Manual PDF** → STS servo register specifications
8. **Fetch Waveshare wiki page** → Online register reference
9. Cross-reference all sources, identify gaps, conflicts, and discrepancies
10. Flag any discrepancies between manufacturer docs and library headers

**Output**: Complete master register table with source tracking and manufacturer validation

### Phase 2: Discrepancy Analysis

**Goal**: Systematically identify and categorize all discrepancies

**Categories**:
- **Type A**: Register exists in library but not in your implementation
- **Type B**: Register exists in your implementation but not in library (potential bug!)
- **Type C**: Register exists in both but used differently (byte order, value encoding)
- **Type D**: Same address, different meanings per servo type (conflicts)
- **Type E**: Register usage patterns differ between library and your code
- **Type F**: Library doesn't match manufacturer documentation (library or doc error)

**Output**: Categorized discrepancy list with severity ratings

### Phase 3: Verification Protocol

**Goal**: For each discrepancy, follow systematic verification steps

**Standard Verification Template** (see MODE register example below):
1. State the discrepancy
2. Cite evidence from each source
3. Identify the conflict/issue
4. Propose resolution
5. Define verification test
6. Document expected behavior
7. Track resolution status

### Phase 4: Implementation & Testing

**Goal**: Fix discrepancies and validate with tests

**Process**:
1. Create tracking document with all discrepancies
2. Prioritize by severity (critical → high → medium → low)
3. Fix code for each discrepancy
4. Write automated test for each fix
5. Run hardware validation tests
6. Update tracking document with resolution status

---

## Example: MODE Register Discrepancy Analysis

### Using Systematic Template

#### 1. Discrepancy Statement
**Register**: MODE (address 33)
**Type**: Type B - Exists in implementation but not for all servo types
**Severity**: CRITICAL - Likely causing current bugs

#### 2. Evidence from Sources

**Source A - SCSCL.h** (SC servo library header):
```
NO MODE REGISTER DEFINED
(Register 33 is not present in the file)
```

**Source B - SMS_STS.h** (STS servo library header):
```cpp
#define SMS_STS_MODE 33  // Line 25
// Comment: 内存表定义 / EPROM(读写)
```

**Source C - SCSCL.cpp** (SC servo library implementation):
```cpp
// Line 79-86: PWMMode() function
int SCSCL::PWMMode(u8 ID) {
    u8 bBuf[4];
    bBuf[0] = 0;  // MIN_ANGLE_LIMIT_L = 0
    bBuf[1] = 0;  // MIN_ANGLE_LIMIT_H = 0
    bBuf[2] = 0;  // MAX_ANGLE_LIMIT_L = 0
    bBuf[3] = 0;  // MAX_ANGLE_LIMIT_H = 0
    return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);
}
// SC servos enter PWM mode by setting angle limits to (0,0)
// DOES NOT use MODE register!
```

**Source D - SMS_STS.cpp** (STS servo library implementation):
```cpp
// Line 77-79: WheelMode() function
int SMS_STS::WheelMode(u8 ID) {
    return writeByte(ID, SMS_STS_MODE, 1);
}
// STS servos use MODE register: 1=wheel mode
// (PWM mode would use MODE=2, position mode uses MODE=0)
```

**Source E - main.cpp** (Your implementation):
```cpp
// Line 58: Register enum
mode = 33,  // Servo mode: 0=position, 1=wheel (continuous rotation)

// Usage: Applied to both SC and STS servos without type checking
// BUG: SC servos don't have MODE register!
```

**Source F - Waveshare Communication Protocol PDF**:
[To be filled after fetching document]

**Source G - ST3215 User Manual PDF**:
[To be filled after fetching document - expected to confirm MODE register for STS]

**Source H - Waveshare ST3215 Wiki**:
[To be filled after fetching page - expected to confirm MODE register usage]

#### 3. Conflict Identification

**The Issue**:
- SC servos (SCSCL series) DO NOT have a MODE register
- SC servos enter PWM mode by writing angle limits to (0, 0)
- STS servos (SMS_STS series) HAVE a MODE register at address 33
- STS servos use MODE register to switch between position/wheel/PWM modes
- Your code uses MODE register for both types → **will fail on SC servos**

**Impact**:
- Writing to address 33 on SC servo may have no effect or undefined behavior
- PWM mode, wheel mode operations likely broken on SC servos
- This is a **Type B discrepancy with CRITICAL severity**

#### 4. Proposed Resolution

**Code Changes Required**:

```cpp
// Add servo type check before mode operations
bool enable_pwm_mode(uint8_t servo_id) {
  if (servo_type_ == ServoType::SC) {
    // SC servos: Write angle limits to (0,0) for PWM mode
    uint8_t buffer[4] = {0, 0, 0, 0};
    return write_bytes(servo_id, Register::min_angle_limit_l, buffer, 4);
  } else {
    // STS servos: Write MODE register = 2
    return write_byte(servo_id, Register::mode, 2);
  }
}

bool enable_wheel_mode(uint8_t servo_id) {
  if (servo_type_ == ServoType::SC) {
    // SC servos: Don't support wheel mode!
    Serial.println("ERROR: SC servos don't support wheel mode");
    return false;
  } else {
    // STS servos: Write MODE register = 1
    return write_byte(servo_id, Register::mode, 1);
  }
}
```

#### 5. Verification Tests

**Test 1: MODE Register Addressing** (Unit Test)
```cpp
void test_mode_register_address() {
  // Verify MODE register only used with STS servos
  TEST_ASSERT_EQUAL(33, to_byte(Register::mode));

  // Verify is_register_valid_for_type rejects MODE for SC
  TEST_ASSERT_FALSE(is_register_valid_for_type(Register::mode, ServoType::SC));
  TEST_ASSERT_TRUE(is_register_valid_for_type(Register::mode, ServoType::STS));
}
```

**Test 2: PWM Mode Activation** (Hardware Integration Test)
```cpp
void test_pwm_mode_sc_servo() {
  // With SC servo ID 2
  bus.set_servo_type(ServoType::SC);

  // Should write to angle limits, not MODE register
  bool result = bus.enable_pwm_mode(2);
  TEST_ASSERT_TRUE(result);

  // Verify can control PWM speed
  result = bus.set_pwm_speed(2, 500);  // Forward at 500
  TEST_ASSERT_TRUE(result);

  // Observe servo rotating continuously
}

void test_pwm_mode_sts_servo() {
  // With STS servo ID 5
  bus.set_servo_type(ServoType::STS);

  // Should write to MODE register = 2
  bool result = bus.enable_pwm_mode(5);
  TEST_ASSERT_TRUE(result);

  // Verify MODE register was written
  uint8_t mode = bus.read_byte(5, Register::mode);
  TEST_ASSERT_EQUAL(2, mode);
}
```

#### 6. Expected Behavior

**SC Servos (ID 2, 3, 4)**:
- `enable_pwm_mode()` → Writes registers 9-12 (angle limits) to 0
- `set_pwm_speed(500)` → Writes to GOAL_TIME register with bit 10 for direction
- MODE register (33) is never accessed

**STS Servos (ID 5)**:
- `enable_pwm_mode()` → Writes register 33 (MODE) = 2
- `enable_wheel_mode()` → Writes register 33 (MODE) = 1
- `restore_position_mode()` → Writes register 33 (MODE) = 0

#### 7. Resolution Status

- [ ] Discrepancy documented in tracking table
- [ ] Code fix implemented in main.cpp
- [ ] Unit tests written and passing
- [ ] Hardware test with SC servo confirms PWM mode works
- [ ] Hardware test with STS servo confirms MODE register works
- [ ] Documentation updated in docs/register_map.md
- [ ] Common mistake documented in docs/common_mistakes.md

---

## Complete Discrepancy List (To Be Populated)

This table will be filled during Phase 2 analysis:

| # | Register | Addr | Type | Severity | Status | Notes |
|---|----------|------|------|----------|--------|-------|
| 1 | MODE | 33 | B | CRITICAL | 🔴 Open | SC servos don't have MODE - use angle limits instead |
| 2 | LOCK | 48 | D | HIGH | ✅ Handled | Conflict: SC=LOCK, STS=TORQUE_LIMIT_L (using lock_sc/lock_sts) |
| 3 | ACC | 41 | - | - | ✅ OK | STS only, correctly guarded in code |
| 4 | OFS_L | 31 | A | MEDIUM | 🔴 Open | Exists in SMS_STS.h but not in enum |
| 5 | OFS_H | 32 | A | MEDIUM | 🔴 Open | Exists in SMS_STS.h but not in enum |
| 6 | TORQUE_LIMIT_L | 48 | A | MEDIUM | 🔴 Open | Exists in SMS_STS.h but conflicts with SC LOCK |
| 7 | TORQUE_LIMIT_H | 49 | A | MEDIUM | 🔴 Open | Exists in SMS_STS.h but not in enum |
| ... | (more to be discovered) | | | | | |

**Legend**:
- Type A: In library, not in implementation
- Type B: In implementation, not in library (or wrong servo type)
- Type C: Different usage patterns
- Type D: Address conflict between servo types
- Status: 🔴 Open, 🟡 In Progress, ✅ Resolved

---

## Critical Issues Identified

### 🚨 High Priority Bugs Found

1. **MODE Register (33) - POTENTIAL BUG**
   - Used in [main.cpp:58](src/main.cpp#L58) for both SC and STS servos
   - **NOT DEFINED in SCSCL.h** - only exists in SMS_STS.h
   - SC servos likely don't support MODE register!
   - SC servos use angle limits (0,0) for PWM mode instead

2. **Register Address 48 Conflict**
   - SC servos: LOCK register
   - STS servos: TORQUE_LIMIT_L register
   - Currently handled correctly with `lock_sc` vs `lock_sts` enums
   - Need to ensure TORQUE_LIMIT not used on SC servos

3. **Missing STS-Specific Registers**
   - OFS_L/H (31-32): Offset calibration - not in enum
   - TORQUE_LIMIT_L/H (48-49): Torque limiting - not in enum

---

## Implementation Plan

### Step 1: Build Master Register Inventory

**File**: `docs/register_inventory.md`

This is the **source of truth** that tracks every register from all sources including manufacturer documentation.

**Table Structure**:
```markdown
| Addr | SC Name (SCSCL.h) | STS Name (SMS_STS.h) | Mfr Doc Name | Mem Type | In Enum? | SC Impl? | STS Impl? | Mfr Match? | Discrepancy | Severity |
```

**Column Definitions**:
- **Addr**: Register address (0-255)
- **SC Name**: Register name from SCSCL.h (or "N/A" if not defined)
- **STS Name**: Register name from SMS_STS.h (or "N/A" if not defined)
- **Mfr Doc Name**: Register name from manufacturer docs (Waveshare PDF, ST3215 manual, wiki)
- **Mem Type**: EPROM-RO, EPROM-RW, SRAM-RO, SRAM-RW, or "Undefined"
- **In Enum?**: Is this register in your main.cpp enum? (✓ / ✗ / ⚠)
- **SC Impl?**: Is it actually used for SC servos? (✓ / ✗ / ?)
- **STS Impl?**: Is it actually used for STS servos? (✓ / ✗ / ?)
- **Mfr Match?**: Does library match manufacturer docs? (✓ / ✗ / ⚠)
- **Discrepancy**: Type (A/B/C/D/E/F) or "None"
- **Severity**: CRITICAL / HIGH / MEDIUM / LOW / OK

**New Discrepancy Type**:
- **Type F**: Library doesn't match manufacturer documentation (potential library bug or doc error)

**Full Register Inventory** (156 entries based on library headers):

| Addr | SC Name | STS Name | Mem Type | In Enum? | SC Impl? | STS Impl? | Discrepancy | Severity |
|------|---------|----------|----------|----------|----------|-----------|-------------|----------|
| 0 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 1 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 2 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 3 | VERSION_L | MODEL_L | EPROM-RO | ✓ | ✓ | ✓ | None | OK |
| 4 | VERSION_H | MODEL_H | EPROM-RO | ✓ | ✓ | ✓ | None | OK |
| 5 | ID | ID | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 6 | BAUD_RATE | BAUD_RATE | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 7 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 8 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 9 | MIN_ANGLE_LIMIT_L | MIN_ANGLE_LIMIT_L | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 10 | MIN_ANGLE_LIMIT_H | MIN_ANGLE_LIMIT_H | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 11 | MAX_ANGLE_LIMIT_L | MAX_ANGLE_LIMIT_L | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 12 | MAX_ANGLE_LIMIT_H | MAX_ANGLE_LIMIT_H | EPROM-RW | ✓ | ✓ | ✓ | None | OK |
| 13-25 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 26 | CW_DEAD | CW_DEAD | EPROM-RW | ✓ | ? | ? | None | OK |
| 27 | CCW_DEAD | CCW_DEAD | EPROM-RW | ✓ | ? | ? | None | OK |
| 28-30 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 31 | N/A | OFS_L | EPROM-RW | ✗ | N/A | ✗ | Type A | MEDIUM |
| 32 | N/A | OFS_H | EPROM-RW | ✗ | N/A | ✗ | Type A | MEDIUM |
| **33** | **N/A** | **MODE** | **EPROM-RW** | **✓** | **⚠** | **✓** | **Type B** | **CRITICAL** |
| 34-39 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 40 | TORQUE_ENABLE | TORQUE_ENABLE | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 41 | N/A | ACC | SRAM-RW | ✓ | N/A | ✓ | None | OK |
| 42 | GOAL_POSITION_L | GOAL_POSITION_L | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 43 | GOAL_POSITION_H | GOAL_POSITION_H | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 44 | GOAL_TIME_L | GOAL_TIME_L | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 45 | GOAL_TIME_H | GOAL_TIME_H | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 46 | GOAL_SPEED_L | GOAL_SPEED_L | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| 47 | GOAL_SPEED_H | GOAL_SPEED_H | SRAM-RW | ✓ | ✓ | ✓ | None | OK |
| **48** | **LOCK** | **TORQUE_LIMIT_L** | **Mixed** | **✓** | **✓** | **✗** | **Type D** | **HIGH** |
| 49 | N/A | TORQUE_LIMIT_H | SRAM-RW | ✗ | N/A | ✗ | Type A | MEDIUM |
| 50-54 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 55 | N/A | LOCK | SRAM-RW | ✓ | N/A | ✓ | None | OK |
| 56 | PRESENT_POSITION_L | PRESENT_POSITION_L | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 57 | PRESENT_POSITION_H | PRESENT_POSITION_H | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 58 | PRESENT_SPEED_L | PRESENT_SPEED_L | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 59 | PRESENT_SPEED_H | PRESENT_SPEED_H | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 60 | PRESENT_LOAD_L | PRESENT_LOAD_L | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 61 | PRESENT_LOAD_H | PRESENT_LOAD_H | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 62 | PRESENT_VOLTAGE | PRESENT_VOLTAGE | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 63 | PRESENT_TEMPERATURE | PRESENT_TEMPERATURE | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 64-65 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 66 | MOVING | MOVING | SRAM-RO | ✓ | ✓ | ✓ | None | OK |
| 67-68 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |
| 69 | PRESENT_CURRENT_L | PRESENT_CURRENT_L | SRAM-RO | ✓ | ? | ? | None | OK |
| 70 | PRESENT_CURRENT_H | PRESENT_CURRENT_H | SRAM-RO | ✓ | ? | ? | None | OK |
| 71-255 | - | - | Undefined | ✗ | ✗ | ✗ | None | - |

**Summary Statistics**:
- Total defined registers (SC): 23
- Total defined registers (STS): 27
- Common registers: 21
- SC-only registers: 2 (VERSION vs MODEL naming, LOCK at 48)
- STS-only registers: 6 (OFS_L/H, MODE, ACC, TORQUE_LIMIT_L/H, LOCK at 55)
- Conflicts: 1 (address 48)
- Type A discrepancies: 3 (OFS_L, OFS_H, TORQUE_LIMIT_H)
- Type B discrepancies: 1 (MODE used on SC when shouldn't be)
- Type D discrepancies: 1 (address 48 conflict)

### Step 2: Create Discrepancy Tracking Document

**File**: `docs/discrepancies.md`

Each discrepancy gets a detailed analysis using the 7-step template:

**Template**:
```markdown
## Discrepancy #X: [Register Name]

### 1. Statement
**Register**: NAME (address XX)
**Type**: A/B/C/D/E/F
**Severity**: CRITICAL/HIGH/MEDIUM/LOW

### 2. Evidence from Sources

**Library Headers:**
- **SCSCL.h**: [quote or "N/A"]
- **SMS_STS.h**: [quote or "N/A"]

**Library Implementation:**
- **SCSCL.cpp**: [implementation details or "N/A"]
- **SMS_STS.cpp**: [implementation details or "N/A"]

**Your Implementation:**
- **main.cpp**: [current usage and line numbers]

**Manufacturer Documentation:**
- **Waveshare Protocol PDF**: [register definition from official protocol spec]
- **ST3215 Manual PDF**: [register spec from datasheet]
- **Waveshare Wiki**: [online reference information]

### 3. Conflict Identification
[Describe the exact problem and its impact]
[Include any conflicts between manufacturer docs and library]

### 4. Proposed Resolution
[Code changes needed]
[If Type F discrepancy, note which source is authoritative]

### 5. Verification Tests
[Specific tests to write]
[If manufacturer docs differ from library, test actual hardware behavior]

### 6. Expected Behavior
[What should happen after fix]
[Reference manufacturer docs for expected behavior]

### 7. Resolution Status
- [ ] Manufacturer docs reviewed
- [ ] Discrepancy documented
- [ ] Code fixed
- [ ] Tests written
- [ ] Hardware validated against manufacturer specs
- [ ] Documentation updated
```

**All Discrepancies to Document**:
1. MODE register (33) - Type B - CRITICAL
2. Address 48 conflict (LOCK vs TORQUE_LIMIT_L) - Type D - HIGH
3. OFS_L (31) missing - Type A - MEDIUM
4. OFS_H (32) missing - Type A - MEDIUM
5. TORQUE_LIMIT_H (49) missing - Type A - MEDIUM
6. (More to be discovered during implementation analysis)

### Step 3: Analyze Implementation for Usage Patterns

**Goal**: Find Type C and Type E discrepancies

**Search for**:
1. All uses of `write_byte()` with register addresses
2. All uses of `write_word()` / `pack_uint16` calls
3. All uses of servo mode switching functions
4. All places where servo_type is checked (or should be but isn't!)

**Look for**:
- Direction encoding (bit 15 vs bit 10 - where is each used?)
- Byte order usage (LE vs BE - is it always correct?)
- Register writes without servo type validation
- Hard-coded register addresses (bypassing enum)

### Step 4: Create Validation Test Suite

**File**: `test/test_register_validation/test_main.cpp`

One test per discrepancy + general validation tests.

**Test Categories**:

#### A. Register Address Validation Tests
Verify custom enum values match library definitions:
```cpp
void test_sc_registers() {
  // Verify SC-specific registers match SCSCL.h
  TEST_ASSERT_EQUAL(48, SCServoBus::to_byte(Register::lock_sc));
}

void test_sts_registers() {
  // Verify STS-specific registers match SMS_STS.h
  TEST_ASSERT_EQUAL(33, SCServoBus::to_byte(Register::mode));
  TEST_ASSERT_EQUAL(41, SCServoBus::to_byte(Register::acc));
  TEST_ASSERT_EQUAL(55, SCServoBus::to_byte(Register::lock_sts));
}

void test_common_registers() {
  // Verify registers that exist on both types
  TEST_ASSERT_EQUAL(5, SCServoBus::to_byte(Register::id));
  TEST_ASSERT_EQUAL(42, SCServoBus::to_byte(Register::goal_position_l));
  // ... all common registers
}
```

#### B. Byte Order Validation Tests
Verify endianness handling:
```cpp
void test_sc_big_endian() {
  uint8_t buffer[2];
  pack_uint16_be(buffer, 0x1234);
  TEST_ASSERT_EQUAL(0x12, buffer[0]);  // HIGH first
  TEST_ASSERT_EQUAL(0x34, buffer[1]);  // LOW second
}

void test_sts_little_endian() {
  uint8_t buffer[2];
  pack_uint16_le(buffer, 0x1234);
  TEST_ASSERT_EQUAL(0x34, buffer[0]);  // LOW first
  TEST_ASSERT_EQUAL(0x12, buffer[1]);  // HIGH second
}
```

#### C. Servo Type Validation Tests
Verify servo-specific features are properly guarded:
```cpp
void test_mode_register_servo_type() {
  // MODE register should only be used on STS servos
  // SC servos don't have MODE register (not in SCSCL.h)
  // This test validates we're not using MODE on SC servos
}

void test_acc_register_sts_only() {
  // Verify ACC (41) only used with STS servos
}

void test_lock_register_correct_address() {
  // Verify we use register 48 for SC, 55 for STS
}
```

#### D. Value Encoding Tests
Verify direction/speed encoding:
```cpp
void test_wheel_mode_direction_encoding() {
  // Wheel mode: bit 15 = direction flag
  // Positive speed = CW, negative = CCW
}

void test_pwm_mode_direction_encoding() {
  // PWM mode: bit 10 = direction flag
  // Different from wheel mode!
}
```

### Step 3: Add Missing Registers to Enum

**File**: `src/main.cpp` (modify Register enum)

Add missing STS-specific registers:
```cpp
enum class Register : uint8_t {
  // ... existing registers ...

  // STS-specific additions
  ofs_l = 31,              // Offset calibration low byte
  ofs_h = 32,              // Offset calibration high byte
  torque_limit_l = 48,     // Torque limit low (conflicts with SC LOCK!)
  torque_limit_h = 49,     // Torque limit high
};
```

### Step 4: Add Runtime Validation

**File**: `src/main.cpp` (add to SCServoBus class)

Add validation helper:
```cpp
static bool is_register_valid_for_type(Register reg, ServoType type) {
  switch(reg) {
    // STS-only registers
    case Register::mode:
    case Register::acc:
    case Register::ofs_l:
    case Register::ofs_h:
    case Register::torque_limit_l:
    case Register::torque_limit_h:
    case Register::lock_sts:
      return type == ServoType::STS;

    // SC-only registers
    case Register::lock_sc:
      return type == ServoType::SC;

    // Common registers
    default:
      return true;
  }
}
```

Add validation to write methods:
```cpp
bool write_byte(uint8_t servo_id, Register reg, uint8_t value) {
  if (!is_register_valid_for_type(reg, servo_type_)) {
    Serial.printf("ERROR: Register %d invalid for %s servo ID %d\n",
                  to_byte(reg),
                  servo_type_ == ServoType::SC ? "SC" : "STS",
                  servo_id);
    last_error_ = ServoError::invalid_parameter;
    return false;
  }
  // ... existing implementation
}
```

### Step 5: Fix MODE Register Bug

**File**: `src/main.cpp`

Current code uses MODE register for both servo types - this is likely wrong!

**Fix**: Update PWM mode functions to use correct method per servo type:
- **SC servos**: Set angle limits to (0, 0) for PWM mode
- **STS servos**: Set MODE register to 2 for PWM mode

Update these functions:
- `enable_pwm_mode()` - currently uses MODE register for both types
- `restore_position_mode_from_pwm()` - currently uses MODE register for both types

### Step 6: Create Bug Prevention Documentation

**File**: `docs/common_mistakes.md`

Document patterns that have caused bugs:

```markdown
# Common Servo Register Mistakes

## 1. Using MODE Register with SC Servos
**Problem**: MODE register (33) doesn't exist on SC servos (not in SCSCL.h)
**Current Bug**: enable_pwm_mode() uses MODE for both types
**Fix**:
- SC servos: Set angle limits to (0, 0) for PWM mode
- STS servos: Use MODE register = 2

## 2. Wrong LOCK Register Address
**Problem**: SC uses 48, STS uses 55
**Fix**: Always use lock_sc vs lock_sts enum values

## 3. Register 48 Conflict
**Problem**: Same address, different meaning per servo type
- SC: LOCK register
- STS: TORQUE_LIMIT_L register
**Fix**: Check servo type before accessing address 48

## 4. Byte Order Confusion
**Problem**: Reading/writing multi-byte values with wrong endianness
**Symptoms**: Position 2048 becomes 8 or 32768
**Fix**:
- SC servos: Always use pack_uint16_be/unpack_uint16_be
- STS servos: Always use pack_uint16_le/unpack_uint16_le

## 5. Using ACC Register on SC Servos
**Problem**: ACC (41) only exists on STS servos
**Fix**: Check servo type before using acceleration features
```

**File**: `docs/byte_order_guide.md`

Quick reference for byte order:
```markdown
# Byte Order Quick Reference

## SC Servos - Big Endian (HIGH, LOW)
Value 0x1234 stored as:
- Address N:   0x12 (HIGH byte)
- Address N+1: 0x34 (LOW byte)

Reading: `value = (buffer[0] << 8) | buffer[1]`

## STS Servos - Little Endian (LOW, HIGH)
Value 0x1234 stored as:
- Address N:   0x34 (LOW byte)
- Address N+1: 0x12 (HIGH byte)

Reading: `value = buffer[0] | (buffer[1] << 8)`

## Functions to Use
- SC: `pack_uint16_be()`, `unpack_uint16_be()`
- STS: `pack_uint16_le()`, `unpack_uint16_le()`
```

---

## Implementation Workflow

### Phase 1: Inventory & Documentation (Days 1-2)

**1.0 Fetch Manufacturer Documentation**
- [ ] Fetch Waveshare Communication Protocol PDF (https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf)
- [ ] Extract register map from protocol PDF
- [ ] Fetch ST3215 User Manual PDF (https://download.kamami.pl/p1181056-ST3215_Servo_User_Manual.pdf)
- [ ] Extract register specifications from ST3215 manual
- [ ] Fetch Waveshare ST3215 Wiki page (https://www.waveshare.com/wiki/ST3215_Servo)
- [ ] Extract register information from wiki
- [ ] Create summary document of manufacturer register specifications

**1.1 Build Register Inventory**
- [ ] Create `docs/register_inventory.md` with full table (including Mfr Doc Name column)
- [ ] Cross-reference all registers from SCSCL.h
- [ ] Cross-reference all registers from SMS_STS.h
- [ ] Cross-reference all registers from manufacturer docs
- [ ] Mark "In Enum?" column for each register
- [ ] Mark "SC Impl?" and "STS Impl?" based on current code analysis
- [ ] Mark "Mfr Match?" column comparing library to manufacturer docs
- [ ] Fill in "Discrepancy" and "Severity" columns
- [ ] Flag any Type F discrepancies (library vs manufacturer conflicts)
- [ ] Add summary statistics at bottom

**1.2 Document All Discrepancies**
- [ ] Create `docs/discrepancies.md`
- [ ] Document MODE register (33) using 7-step template
- [ ] Document address 48 conflict using 7-step template
- [ ] Document OFS_L (31) missing using 7-step template
- [ ] Document OFS_H (32) missing using 7-step template
- [ ] Document TORQUE_LIMIT_H (49) missing using 7-step template
- [ ] Search for and document any additional discrepancies found

**1.3 Create Reference Documentation**
- [ ] Create `docs/common_mistakes.md` with bug patterns
- [ ] Create `docs/byte_order_guide.md` with SC vs STS reference
- [ ] Create `docs/value_encoding_guide.md` for direction bits, etc.

### Phase 2: Code Analysis & Pattern Discovery (Day 3)

**2.1 Systematic Code Search**
- [ ] Grep for all `write_byte(` calls - verify register usage
- [ ] Grep for all `write_word(` calls - verify byte order
- [ ] Grep for all `pack_uint16_be` calls - verify used only with SC
- [ ] Grep for all `pack_uint16_le` calls - verify used only with STS
- [ ] Grep for all register accesses - check servo type validation
- [ ] Grep for bit shifts (<<) - verify direction encoding patterns
- [ ] List all functions that should check servo type but don't

**2.2 Document Findings**
- [ ] Add any Type C discrepancies to docs/discrepancies.md
- [ ] Add any Type E discrepancies to docs/discrepancies.md
- [ ] Update register_inventory.md with new findings
- [ ] Prioritize all discrepancies by severity

### Phase 3: Code Fixes (Days 4-5)

**3.1 Fix CRITICAL Discrepancies**
- [ ] Fix MODE register usage in `enable_pwm_mode()`
  - Add servo type check
  - SC servos: Write angle limits to (0,0)
  - STS servos: Write MODE = 2
- [ ] Fix MODE register usage in `enable_wheel_mode()`
  - Add servo type check
  - SC servos: Error (not supported)
  - STS servos: Write MODE = 1
- [ ] Fix MODE register usage in `restore_position_mode()`
  - Add servo type check
  - SC servos: Restore angle limits from stored values
  - STS servos: Write MODE = 0

**3.2 Add Missing Registers**
- [ ] Add `ofs_l = 31` to Register enum
- [ ] Add `ofs_h = 32` to Register enum
- [ ] Add `torque_limit_l = 48` to Register enum (note conflict!)
- [ ] Add `torque_limit_h = 49` to Register enum

**3.3 Add Runtime Validation**
- [ ] Implement `is_register_valid_for_type()` helper
- [ ] Add validation to `write_byte()` method
- [ ] Add validation to `write_word()` / multi-byte methods
- [ ] Add helpful error messages for invalid register usage

**3.4 Fix Address 48 Conflict**
- [ ] Ensure lock_sc (48) only used with SC servos
- [ ] Ensure torque_limit_l (48) only used with STS servos
- [ ] Add runtime check to prevent cross-usage

### Phase 4: Testing (Days 6-7)

**4.1 Create Test Suite**
- [ ] Create `test/test_register_validation/test_main.cpp`
- [ ] Configure PlatformIO test environment
- [ ] Write register address validation tests (compare enum to library)
- [ ] Write byte order validation tests (BE vs LE)
- [ ] Write servo type validation tests (register type checking)
- [ ] Write value encoding tests (direction bits)

**4.2 Unit Testing (No Hardware)**
- [ ] Run all register address tests
- [ ] Run all byte order tests
- [ ] Run all servo type validation logic tests
- [ ] Fix any failures

**4.3 Hardware Integration Testing**
- [ ] Test MODE register fix: PWM mode on SC servo ID 2
- [ ] Test MODE register fix: PWM mode on STS servo ID 5
- [ ] Test MODE register fix: Wheel mode on STS servo ID 5
- [ ] Test LOCK register: Write/read on SC servo (addr 48)
- [ ] Test LOCK register: Write/read on STS servo (addr 55)
- [ ] Test byte order: Position control on SC servo (big-endian)
- [ ] Test byte order: Position control on STS servo (little-endian)
- [ ] Test ACC register: Only works on STS servo
- [ ] Test runtime validation: Attempt invalid register usage
- [ ] Document all test results

### Phase 5: Documentation & Review (Day 8)

**5.1 Update Tracking Documents**
- [ ] Update docs/discrepancies.md with resolution status
- [ ] Update docs/register_inventory.md with final implementation status
- [ ] Mark all resolved discrepancies as ✅
- [ ] Add lessons learned section

**5.2 Final Documentation**
- [ ] Update readme.md with links to new docs
- [ ] Add "Register Validation" section to readme
- [ ] Document the systematic process for future use
- [ ] Create quick reference card for register usage

---

## Success Criteria

✅ All registers documented in tracking table
✅ Implementation status clear for each register
✅ Servo-specific features clearly marked
✅ MODE register bug fixed for SC servos
✅ Missing registers added to enum
✅ Runtime validation prevents wrong register usage
✅ Automated tests validate register addresses
✅ Automated tests validate byte order handling
✅ Clear documentation prevents future bugs

---

## Files to Create/Modify

### New Documentation Files
1. **`docs/manufacturer_specs.md`** - Manufacturer documentation summary
   - Extracted register maps from Waveshare Protocol PDF
   - Register specifications from ST3215 User Manual
   - Reference information from Waveshare wiki
   - Discrepancies between manufacturer sources
   - Authoritative source determination

2. **`docs/register_inventory.md`** - Master register inventory (all addresses 0-255)
   - Complete cross-reference of SC and STS registers
   - Cross-reference with manufacturer documentation
   - Implementation status tracking
   - Manufacturer documentation match status
   - Discrepancy and severity columns

3. **`docs/discrepancies.md`** - Detailed discrepancy analysis
   - Each discrepancy documented with 7-step template
   - Evidence from all sources (library + manufacturer docs)
   - Manufacturer documentation validation
   - Proposed fixes and test plans
   - Resolution tracking

4. **`docs/common_mistakes.md`** - Bug prevention guide
   - Real-world bug patterns from your experience
   - Common register confusion scenarios
   - Solutions and best practices
   - References to manufacturer documentation

5. **`docs/byte_order_guide.md`** - Endianness quick reference
   - SC vs STS byte order differences
   - When to use BE vs LE functions
   - Common byte order mistakes
   - Verified against manufacturer specs

6. **`docs/value_encoding_guide.md`** - Value encoding reference
   - Direction bit encoding (bit 15 for wheel, bit 10 for PWM)
   - Speed/position encoding patterns
   - Mode values (0=position, 1=wheel, 2=PWM)
   - Validated against manufacturer documentation

### Code Files to Modify
1. **`src/main.cpp`**
   - Add missing registers to enum (ofs_l, ofs_h, torque_limit_l, torque_limit_h)
   - Add `is_register_valid_for_type()` helper function
   - Add runtime validation to write methods
   - **FIX CRITICAL BUG**: MODE register usage in enable_pwm_mode()
   - **FIX CRITICAL BUG**: MODE register usage in enable_wheel_mode()
   - **FIX CRITICAL BUG**: MODE register usage in restore_position_mode()
   - Fix address 48 conflict handling

### Test Files to Create
1. **`test/test_register_validation/test_main.cpp`**
   - Register address validation tests
   - Byte order validation tests
   - Servo type validation tests
   - Value encoding tests
   - Hardware integration tests

### Reference Files (Read-Only)
1. `.pio/libdeps/.../SCServo/src/SCSCL.h` - SC register definitions (23 registers)
2. `.pio/libdeps/.../SCServo/src/SMS_STS.h` - STS register definitions (27 registers)
3. `.pio/libdeps/.../SCServo/src/SCSCL.cpp` - SC implementation patterns
4. `.pio/libdeps/.../SCServo/src/SMS_STS.cpp` - STS implementation patterns
5. `.pio/libdeps/.../SCServo/src/INST.h` - Instruction definitions

---

## Summary: What Makes This Plan Systematic

This plan provides a **repeatable, methodical process** that prevents the types of bugs you've been experiencing:

### 1. Complete Inventory
- Every register from 0-255 tracked in master table
- Cross-referenced against official library headers
- Implementation status clearly marked

### 2. Categorized Discrepancies
- Type A: Missing from implementation
- Type B: In implementation but shouldn't be (or wrong type)
- Type C: Different usage patterns
- Type D: Address conflicts between servo types
- Type E: Implementation vs library behavior differences

### 3. Evidence-Based Analysis
- Every discrepancy backed by citations from source files
- Line numbers and code quotes provided
- No guessing - everything verified against official library

### 4. Systematic Documentation
- 7-step template ensures nothing is missed
- Trackable resolution status
- Clear verification criteria

### 5. Multi-Layer Validation
- Compile-time: Enum definitions
- Runtime: Type checking in write methods
- Unit tests: Logic validation
- Hardware tests: Real-world verification

### 6. Future-Proof Process
- Documented methodology can be reused
- New registers/servos can be added systematically
- Clear process for validating against updates

### 7. Manufacturer Documentation Validation
- Cross-reference with official Waveshare protocol specification
- Cross-reference with ST3215 servo datasheet
- Cross-reference with online wiki documentation
- Identify any discrepancies between library and manufacturer docs
- Use manufacturer docs as authoritative source when conflicts exist
- Validate library assumptions against real hardware behavior

---

## Critical Bugs Identified

### 🚨 CRITICAL Priority

**MODE Register Bug** (Discrepancy #1)
- **Problem**: CODE uses MODE register for SC servos, but SCSCL.h doesn't define it
- **Impact**: PWM mode and wheel mode likely broken on SC servos
- **Location**: [main.cpp:58](src/main.cpp#L58), enable_pwm_mode(), enable_wheel_mode()
- **Fix**: Add servo type checks, use angle limits (0,0) for SC PWM mode
- **Test**: Hardware validation on SC servo ID 2

### ⚠️ HIGH Priority

**Address 48 Conflict** (Discrepancy #2)
- **Problem**: Same address means LOCK (SC) vs TORQUE_LIMIT_L (STS)
- **Impact**: Risk of writing wrong register if servo type not checked
- **Status**: Partially handled with lock_sc/lock_sts enums, needs validation
- **Fix**: Add runtime checks, implement TORQUE_LIMIT support for STS
- **Test**: Verify LOCK on both servo types, test TORQUE_LIMIT on STS

### 📋 MEDIUM Priority

**Missing STS Features** (Discrepancies #3-5)
- OFS_L/OFS_H (31-32): Offset calibration not implemented
- TORQUE_LIMIT_H (49): Torque limiting not implemented
- **Impact**: Missing functionality for STS servos
- **Fix**: Add to enum, implement access functions if needed
- **Test**: Document whether these features are needed for your use case

---

## Next Steps

When you approve this plan, we will execute in phases:

1. **Start with Documentation** (Phase 1) - Build the tracking infrastructure
2. **Analyze Current Code** (Phase 2) - Find all discrepancies systematically
3. **Fix Critical Bugs** (Phase 3) - MODE register is highest priority
4. **Validate with Tests** (Phase 4) - Ensure fixes work on real hardware
5. **Complete Documentation** (Phase 5) - Leave a clear trail for future

The MODE register bug is the **most urgent** - it's likely causing issues with your SC servos right now.
