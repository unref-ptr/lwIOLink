// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#pragma once

// Mock Arduino.h for unit testing
#include <stdint.h>
#include <cstring>
#include <iostream>

// Forward declaration
typedef void (*InterruptCallback)(void);

// Simple mock pin state management using arrays instead of std::map
class MockPinManager {
private:
    static const uint8_t MAX_PINS = 64;
    static uint8_t pinModes[MAX_PINS];
    static uint8_t pinValues[MAX_PINS];
    static unsigned long startTime;
    static bool timeInitialized;
    static unsigned long currentTime;
    static InterruptCallback interruptCallbacks[MAX_PINS];
    static bool interruptEnabled[MAX_PINS];
    static bool interruptTriggered[MAX_PINS];
    
public:
    static void setPinMode(uint8_t pin, uint8_t mode) {
        if (pin < MAX_PINS) {
            pinModes[pin] = mode;
        }
    }
    
    static void setPinValue(uint8_t pin, uint8_t value) {
        if (pin < MAX_PINS) {
            pinValues[pin] = value;
        }
    }
    
    static uint8_t getPinMode(uint8_t pin) {
        return (pin < MAX_PINS) ? pinModes[pin] : 0;
    }
    
    static uint8_t getPinValue(uint8_t pin) {
        return (pin < MAX_PINS) ? pinValues[pin] : 0;
    }
    
    static unsigned long getMicros() {
        if (!timeInitialized) {
            startTime = 0;
            currentTime = 0;
            timeInitialized = true;
        }
        return currentTime;
    }
    
    static void setMicros(unsigned long micros) {
        currentTime = micros;
        timeInitialized = true;
    }
    
    static void advanceTime(unsigned long microseconds) {
        if (!timeInitialized) {
            startTime = 0;
            currentTime = 0;
            timeInitialized = true;
        }
        currentTime += microseconds;
    }
    
    static void resetTime() {
        timeInitialized = false;
        currentTime = 0;
        startTime = 0;
    }
    
    static void setInterruptCallback(uint8_t pin, InterruptCallback callback) {
        if (pin < MAX_PINS) {
            interruptCallbacks[pin] = callback;
            interruptEnabled[pin] = true;
        }
    }
    
    static void triggerInterrupt(uint8_t pin) {
        if (pin < MAX_PINS && interruptEnabled[pin] && interruptCallbacks[pin]) {
            interruptCallbacks[pin]();
            // Track that this pin had an interrupt triggered
            interruptTriggered[pin] = true;
        }
    }
    
    static void clearInterrupt(uint8_t pin) {
        if (pin < MAX_PINS) {
            interruptCallbacks[pin] = nullptr;
            interruptEnabled[pin] = false;
            interruptTriggered[pin] = false;
        }
    }
    
    static bool isInterruptEnabled(uint8_t pin) {
        return (pin < MAX_PINS) ? interruptEnabled[pin] : false;
    }
    
    static bool wasInterruptTriggered(uint8_t pin) {
        return (pin < MAX_PINS) ? interruptTriggered[pin] : false;
    }
    
    static void clearInterruptFlags() {
        for (uint8_t i = 0; i < MAX_PINS; i++) {
            interruptTriggered[i] = false;
        }
    }
    
    // IO-Link specific simulation
    static void simulateWakeupSignal(uint8_t wakeupPin = 2) {
        // Simulate the physical wakeup signal by setting the triggered flag directly
        // This represents the physical event, regardless of callbacks
        if (wakeupPin < MAX_PINS) {
            interruptTriggered[wakeupPin] = true;
        }
        
        // Also trigger any registered callback (for compatibility)
        triggerInterrupt(wakeupPin);
    }
    
    static void reset() {
        for (uint8_t i = 0; i < MAX_PINS; i++) {
            pinModes[i] = 0;
            pinValues[i] = 0;
            interruptCallbacks[i] = nullptr;
            interruptEnabled[i] = false;
            interruptTriggered[i] = false;
        }
        resetTime();
    }
};

// Mock Stream class
class Stream {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual void flush() = 0;
};

// Mock Serial classes for testing
class MockSerial : public Stream {
private:
    uint8_t rxBuffer[256];
    uint8_t txBuffer[256];
    uint8_t rxHead = 0;
    uint8_t rxTail = 0;
    uint8_t txHead = 0;
    uint8_t txTail = 0;
    
public:
    int available() override { 
        return (rxHead >= rxTail) ? (rxHead - rxTail) : (256 - rxTail + rxHead);
    }
    
    int read() override { 
        if (rxHead == rxTail) return -1;
        uint8_t data = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % 256;
        return data;
    }
    
    size_t write(uint8_t data) override { 
        txBuffer[txHead] = data;
        txHead = (txHead + 1) % 256;
        return 1; 
    }
    
    void flush() override {}
    
    // Test utilities
    void addToRxBuffer(uint8_t data) {
        rxBuffer[rxHead] = data;
        rxHead = (rxHead + 1) % 256;
    }
    
    void addToRxBuffer(const uint8_t* data, uint8_t length) {
        for (uint8_t i = 0; i < length; i++) {
            addToRxBuffer(data[i]);
        }
    }
    
    uint8_t getTxBufferByte(uint8_t index) {
        return txBuffer[index];
    }
    
    uint8_t getTxBufferLength() {
        return (txHead >= txTail) ? (txHead - txTail) : (256 - txTail + txHead);
    }
    
    void clearRxBuffer() {
        rxHead = rxTail = 0;
    }
    
    void clearTxBuffer() {
        txHead = txTail = 0;
    }
    
    void clearBuffers() {
        clearRxBuffer();
        clearTxBuffer();
    }
};

// Mock pin functions
inline void pinMode(uint8_t pin, uint8_t mode) {
    MockPinManager::setPinMode(pin, mode);
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
    MockPinManager::setPinValue(pin, value);
}

inline uint8_t digitalRead(uint8_t pin) {
    return MockPinManager::getPinValue(pin);
}

inline void attachInterrupt(uint8_t pin, void (*callback)(), int mode) {
    MockPinManager::setInterruptCallback(pin, callback);
}

inline unsigned long micros() {
    return MockPinManager::getMicros();
}

inline unsigned long millis() {
    return micros() / 1000;
}

inline void delay(unsigned long ms) {
    MockPinManager::advanceTime(ms * 1000); // Convert ms to microseconds
}

inline void delayMicroseconds(unsigned long us) {
    MockPinManager::advanceTime(us);
}

inline int digitalPinToInterrupt(uint8_t pin) {
    return pin;
}

// Constants
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define FALLING 2
#define RISING 3

// Serial configuration constants
#define SERIAL_8E1 0x26

// Global mock serial instance
extern MockSerial Serial;
