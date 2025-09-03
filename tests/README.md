# lwIOLink Unit Testing

## Structure

```
tests/
├── src/                   # Test source files
├── mock/                  # Mock implementations
│   ├── Arduino.h/.cpp     # Arduino hardware abstraction mock
│   └── master/            # IO-Link master simulation utilities
├── build/                 # Build output directory
├── CMakeLists.txt         # CMake configuration
└── README.md              # This file
```

## Building Tests

### CMake (Recommended)
```bash
cd tests
cmake -B build -G Ninja
cmake --build build
./build/lwIOLink_tests
```