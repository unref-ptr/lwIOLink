# Mock Implementations

Hardware abstraction layer mocks for unit testing.

## Components

### Arduino.h/.cpp
Mock Arduino hardware interface providing:
- `Stream` virtual base class for serial communication
- `MockSerial` implementation with buffer management
- `MockPinManager` for GPIO simulation and interrupt handling
- Pin manipulation functions (`pinMode`, `digitalWrite`, etc.)
- Timing functions (`micros`, `millis`, `delay`, etc.)
- Interrupt management (`attachInterrupt`, etc.)
- Arduino constants (`HIGH`, `LOW`, `INPUT_PULLUP`, etc.)

### master/FrameBuilder.h/.cpp
IO-Link master simulation utilities:
- Master command frame construction
- DP1 parameter frame building
- Protocol-compliant frame formatting
- Checksum calculation and validation

## Implementation Notes

- All functions declared `inline` to prevent linking conflicts
- Mock implementations return safe defaults or perform no-ops
- Virtual interfaces enable proper polymorphic testing
- Buffer management supports realistic data flow simulation
- Pin state tracking enables interrupt and GPIO testing
