# Error Handling Strategy

## Design Principles
- **No magic numbers** (no -1, no special return values)
- **Fast by default** (minimal overhead on success path)
- **Detailed diagnostics available** when needed

## Error Types
```cpp
enum class ServoError {
  None = 0,
  Timeout,
  InvalidHeader,
  ChecksumMismatch,
  InvalidResponse,
  InvalidParameter,
  NoAck
};
```

## Return Type Rules

### Commands (write operations)
**Return `bool`** - simple success/failure
```cpp
bool write_pos(uint8_t id, uint16_t position, ...);
bool set_torque(uint8_t id, bool enable);
```

### Queries (read operations)
**Return `std::optional<T>`** - value or nothing
```cpp
std::optional<int> read_position(uint8_t id);
std::optional<uint16_t> read_voltage(uint8_t id);
std::optional<uint8_t> ping(uint8_t id);
```

### Error State
**Always available via member functions:**
```cpp
bool ok() const;                    // Quick check: no error
ServoError last_error() const;      // Get detailed error
void clear_error();                 // Reset error state
```

## Usage Patterns

### Fast path (ignore errors)
```cpp
servo_bus.write_pos(1, 512, 0, 300);
auto pos = servo_bus.read_position(1).value_or(512);
```

### Check success
```cpp
if (!servo_bus.write_pos(1, 512, 0, 300)) {
  // handle failure
}

if (auto pos = servo_bus.read_position(1)) {
  // use *pos
}
```

### Detailed diagnostics
```cpp
if (!servo_bus.write_pos(1, 512, 0, 300)) {
  switch(servo_bus.last_error()) {
    case ServoError::Timeout: /* ... */ break;
    case ServoError::NoAck: /* ... */ break;
  }
}
```

## Performance
- `std::optional<T>` is zero-cost abstraction
- Error state is single byte
- No exceptions, no allocations
- Fast path is just a bool/optional check
