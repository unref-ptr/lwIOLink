// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "CppUTest/TestHarness.h"
#include "lwIOLink.hpp" 
#include "Arduino.h"

TEST_GROUP(InterruptTests)
{
    lwIOLink::Device* device;
    static constexpr uint8_t TEST_WAKEUP_PIN = 2;
    static constexpr uint8_t TEST_TXEN_PIN = 4;

    void setup()
    {
        MockPinManager::reset();
        device = new lwIOLink::Device(1, 1, 1000);
    }
    
    void teardown()
    {
        delete device;
        MockPinManager::reset();
    }
};

TEST(InterruptTests, MockInterruptSystem) 
{
    // Test that mock interrupt system works independently 
    CHECK_EQUAL(false, MockPinManager::wasInterruptTriggered(TEST_WAKEUP_PIN));
    
    // Directly trigger interrupt flag
    MockPinManager::simulateWakeupSignal(TEST_WAKEUP_PIN);
    CHECK_EQUAL(true, MockPinManager::wasInterruptTriggered(TEST_WAKEUP_PIN));
    
    // Clear and test again
    MockPinManager::clearInterruptFlags();
    CHECK_EQUAL(false, MockPinManager::wasInterruptTriggered(TEST_WAKEUP_PIN));
}

TEST(InterruptTests, DeviceWakeupIntegration)
{
    lwIOLink::Device::HWConfig config = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = TEST_TXEN_PIN,
            .Wakeup = TEST_WAKEUP_PIN
        }
    };
    
    device->begin(config);
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    // Test that device registered interrupt
    CHECK_EQUAL(true, MockPinManager::isInterruptEnabled(TEST_WAKEUP_PIN));
    
    // Test wakeup signal detection
    CHECK_EQUAL(false, MockPinManager::wasInterruptTriggered(TEST_WAKEUP_PIN));
    MockPinManager::simulateWakeupSignal(TEST_WAKEUP_PIN);
    CHECK_EQUAL(true, MockPinManager::wasInterruptTriggered(TEST_WAKEUP_PIN));
}
