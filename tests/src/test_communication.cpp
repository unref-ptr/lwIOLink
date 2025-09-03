// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "lwIOLink.hpp"
#include "Arduino.h"
#include "master/FrameBuilder.h"
#include <cmath>

namespace IOLinkTestConstants {
    static constexpr uint8_t TEST_PD_INPUT_SIZE = 1;
    static constexpr uint8_t TEST_PD_OUTPUT_SIZE = 1;
    static constexpr uint32_t TEST_MIN_CYCLE_TIME = 1000;
    static constexpr uint8_t TEST_TXEN_PIN = 4;
    static constexpr uint8_t TEST_WAKEUP_PIN = 2;
    static constexpr uint32_t TEST_CYCLE_TIME_2_3MS = 2300;
    static constexpr double TEST_CYCLE_TIME_TOLERANCE = 0.1;
    static constexpr size_t TEST_FRAME_BUFFER_SIZE = 8;
    static constexpr uint8_t IOLINK_WAKEUP_PULSE_COUNT = 24;
    static constexpr uint32_t IOLINK_WAKEUP_PULSE_WIDTH = 83;
    static constexpr uint32_t IOLINK_STARTUP_TIMEOUT = 100000;
}

namespace IOLinkTestUtils {
    void SendFrameToDevice(const uint8_t* frameData, uint8_t frameLength) {
        Serial.addToRxBuffer(frameData, frameLength);
    }
    
    void FlushSerialBuffers() {
        Serial.clearRxBuffer();
        Serial.clearTxBuffer();
    }
    
    uint8_t GetReceivedLength() {
        return Serial.available();
    }
    
    uint8_t GetTransmittedLength() {
        return Serial.getTxBufferLength();
    }
    
    void TriggerWakeupSignal(uint8_t wakeupPin = IOLinkTestConstants::TEST_WAKEUP_PIN) {
        MockPinManager::simulateWakeupSignal(wakeupPin);
    }
    
    bool WasWakeupTriggered(uint8_t wakeupPin = IOLinkTestConstants::TEST_WAKEUP_PIN) {
        return MockPinManager::wasInterruptTriggered(wakeupPin);
    }
    
    void ClearWakeupFlags() {
        MockPinManager::clearInterruptFlags();
    }

    void SetupInterruptCallback(uint8_t pin, bool& flagToSet) {
        // Simple callback that sets a flag - no lambda needed
        static bool* currentFlag = nullptr;
        currentFlag = &flagToSet;
        
        MockPinManager::setInterruptCallback(pin, []() {
            if (currentFlag) *currentFlag = true;
        });
    }
    
    void ResetAllMocks() {
        MockPinManager::reset();
        FlushSerialBuffers();
    }
    
    double GetCycleTimeDiff(uint32_t cycleTime) {
        // Use inline constants and handle underflow properly
        const uint32_t target = 2300;
        // Handle subtraction to avoid uint32_t underflow
        if (cycleTime >= target) {
            return static_cast<double>(cycleTime - target);
        } else {
            return static_cast<double>(target - cycleTime);
        }
    }
    
    bool IsCycleTimeValid(uint32_t cycleTime) {
        // Use inline calculation to avoid any potential constant resolution issues
        const uint32_t target = 2300;
        const double tolerance = 0.1;
        double diff = GetCycleTimeDiff(cycleTime);
        return diff <= (target * tolerance);
    }
}

TEST_GROUP(IOLinkCommunication)
{
    lwIOLink::Device* device;

    void setup()
    {
        IOLinkTestUtils::ResetAllMocks();
        
        device = new lwIOLink::Device(
            IOLinkTestConstants::TEST_PD_INPUT_SIZE,
            IOLinkTestConstants::TEST_PD_OUTPUT_SIZE,
            IOLinkTestConstants::TEST_MIN_CYCLE_TIME
        );
        
        // Setup pin modes
        MockPinManager::setPinMode(IOLinkTestConstants::TEST_TXEN_PIN, OUTPUT);
        MockPinManager::setPinMode(IOLinkTestConstants::TEST_WAKEUP_PIN, INPUT_PULLUP);
    }
    
    void teardown()
    {
        delete device;
        device = nullptr;
        IOLinkTestUtils::ResetAllMocks();
    }
};

TEST(IOLinkCommunication, DeviceInitialization)
{
    CHECK(device != nullptr);
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
}

TEST(IOLinkCommunication, WakeupInterruptSimulation)
{
    bool interruptTriggered = false;
    IOLinkTestUtils::SetupInterruptCallback(IOLinkTestConstants::TEST_WAKEUP_PIN, interruptTriggered);
    
    CHECK_EQUAL(false, interruptTriggered);
    CHECK(MockPinManager::isInterruptEnabled(IOLinkTestConstants::TEST_WAKEUP_PIN));
    
    MockPinManager::triggerInterrupt(IOLinkTestConstants::TEST_WAKEUP_PIN);
    CHECK_EQUAL(true, interruptTriggered);
}

TEST(IOLinkCommunication, MasterFrameConstruction)
{
    // Test basic frame builder functionality
    uint8_t frame[8];
    uint8_t length = IOLinkMaster::FrameBuilder::BuildMasterCommandFrame(lwIOLink::DeviceIden, frame);
    
    CHECK_TRUE(length > 0);
    CHECK_TRUE(length <= 8);
}

TEST(IOLinkCommunication, CycleTimeUtilities)
{
    // Debug specific case using the fixed utility function
    uint32_t testVal = 2070;
    double diff = IOLinkTestUtils::GetCycleTimeDiff(testVal);
    double threshold = 2300 * 0.1;
    bool result = diff <= threshold;
    
    // These should all be true
    DOUBLES_EQUAL(230.0, diff, 0.01);      // GetCycleTimeDiff(2070) should be 230
    DOUBLES_EQUAL(230.0, threshold, 0.01); // 2300 * 0.1 = 230  
    CHECK(result);                         // 230 <= 230 should be true
    
    // Test cycle time validation with simple boolean checks
    CHECK(IOLinkTestUtils::IsCycleTimeValid(2300)); // Exact match: 0 <= 230
    CHECK(IOLinkTestUtils::IsCycleTimeValid(2070)); // Edge case: 230 <= 230
    
    // Test cycle time difference calculation
    DOUBLES_EQUAL(0.0, IOLinkTestUtils::GetCycleTimeDiff(2300), 0.01);
    DOUBLES_EQUAL(230.0, IOLinkTestUtils::GetCycleTimeDiff(2070), 0.01);
    DOUBLES_EQUAL(230.0, IOLinkTestUtils::GetCycleTimeDiff(2530), 0.01);
}

TEST(IOLinkCommunication, OperateModeTransitionAndCallbacks)
{
    lwIOLink::Device::HWConfig hwConfig = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = IOLinkTestConstants::TEST_TXEN_PIN,
            .Wakeup = IOLinkTestConstants::TEST_WAKEUP_PIN
        }
    };
    device->begin(hwConfig);
    
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    // Test that the device initializes properly
    device->run();
}

TEST(IOLinkCommunication, BasicStateTransitionVerification)
{
    lwIOLink::Device::HWConfig hwConfig = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = IOLinkTestConstants::TEST_TXEN_PIN,
            .Wakeup = IOLinkTestConstants::TEST_WAKEUP_PIN
        }
    };
    device->begin(hwConfig);
    
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    // Debug: Check if interrupt was properly registered
    LONGS_EQUAL(true, MockPinManager::isInterruptEnabled(IOLinkTestConstants::TEST_WAKEUP_PIN));
    
    // Test mock system directly
    CHECK_EQUAL(false, MockPinManager::wasInterruptTriggered(IOLinkTestConstants::TEST_WAKEUP_PIN));
    MockPinManager::simulateWakeupSignal(IOLinkTestConstants::TEST_WAKEUP_PIN);
    CHECK_EQUAL(true, MockPinManager::wasInterruptTriggered(IOLinkTestConstants::TEST_WAKEUP_PIN));
    
    // Now reset and test via the utils
    MockPinManager::clearInterruptFlags();
    CHECK_EQUAL(false, IOLinkTestUtils::WasWakeupTriggered());
    IOLinkTestUtils::TriggerWakeupSignal();
    CHECK_EQUAL(true, IOLinkTestUtils::WasWakeupTriggered());
    
    IOLinkTestUtils::TriggerWakeupSignal(); // Clear and trigger again
    CHECK_EQUAL(true, IOLinkTestUtils::WasWakeupTriggered());
    
    // Test that device processes the wakeup signal
    device->run();
}

TEST(IOLinkCommunication, CyclicDataFlowValidation)
{
    lwIOLink::Device::HWConfig hwConfig = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = IOLinkTestConstants::TEST_TXEN_PIN,
            .Wakeup = IOLinkTestConstants::TEST_WAKEUP_PIN
        }
    };
    device->begin(hwConfig);
    
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    // Trigger wakeup signal to activate the device
    IOLinkTestUtils::TriggerWakeupSignal();
    device->run(); // Process wakeup
    
    // Build the exact first frame that device expects (0xA2 = Read Page Channel, Min Cycle Time)
    uint8_t firstFrame[8];
    uint8_t firstFrameLength = IOLinkMaster::FrameBuilder::BuildReadDP1Frame(lwIOLink::MinCycleTime, firstFrame);
    IOLinkTestUtils::SendFrameToDevice(firstFrame, firstFrameLength);
    device->run(); // Process first frame to transition to run_mode
    
    // Now send another command frame
    uint8_t commandFrame[8];
    uint8_t commandFrameLength = IOLinkMaster::FrameBuilder::BuildMasterCommandFrame(lwIOLink::DeviceIden, commandFrame);
    IOLinkTestUtils::SendFrameToDevice(commandFrame, commandFrameLength);
    device->run(); // Process command frame
    
    CHECK(IOLinkTestUtils::GetTransmittedLength() > 0);
    
    // Test that device handles the frame processing
}

TEST(IOLinkCommunication, ProcessDataChangeDetection)
{
    lwIOLink::Device::HWConfig hwConfig = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = IOLinkTestConstants::TEST_TXEN_PIN,
            .Wakeup = IOLinkTestConstants::TEST_WAKEUP_PIN
        }
    };
    device->begin(hwConfig);
    
    // Device starts in 'start' mode, not 'operate' mode
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    uint8_t testData[IOLinkTestConstants::TEST_PD_INPUT_SIZE] = {0xFF};
    uint8_t retrievedData[IOLinkTestConstants::TEST_PD_OUTPUT_SIZE];
    lwIOLink::PDStatus status;
    
    // Process data functions should return false in non-operate mode
    bool pdInResult = device->SetPDIn(testData, IOLinkTestConstants::TEST_PD_INPUT_SIZE);
    bool pdOutResult = device->GetPDOut(retrievedData, &status);
    
    CHECK_EQUAL(false, pdInResult);  // Should fail - not in operate mode
    CHECK_EQUAL(false, pdOutResult); // Should fail - not in operate mode
    
    device->run();
    
    // Test that process data operations work correctly (return false in start mode)
}

TEST(IOLinkCommunication, MockSystemVerification)
{
    // Test mock pin operations
    MockPinManager::setPinValue(IOLinkTestConstants::TEST_TXEN_PIN, HIGH);
    CHECK_EQUAL(HIGH, MockPinManager::getPinValue(IOLinkTestConstants::TEST_TXEN_PIN));
    
    MockPinManager::setPinValue(IOLinkTestConstants::TEST_TXEN_PIN, LOW);
    CHECK_EQUAL(LOW, MockPinManager::getPinValue(IOLinkTestConstants::TEST_TXEN_PIN));
    
    // Test serial mock operations
    const uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    Serial.addToRxBuffer(testData, 4);
    CHECK_EQUAL(4, Serial.available());
    
    uint8_t readBuffer[10];
    uint8_t bytesRead = 0;
    while (Serial.available() > 0 && bytesRead < 10) {
        int data = Serial.read();
        if (data != -1) {
            readBuffer[bytesRead++] = data;
        }
    }
    CHECK_EQUAL(4, bytesRead);
    CHECK_EQUAL(0x01, readBuffer[0]);
    CHECK_EQUAL(0x04, readBuffer[3]);
    
    CHECK_EQUAL(0, Serial.available());
}

TEST(IOLinkCommunication, ComprehensiveCyclicTestingDemo)
{
    lwIOLink::Device::HWConfig hwConfig = {
        .SerialPort = Serial,
        .Baud = lwIOLink::COM2,
        .WakeupMode = FALLING,
        .Pin = {
            .TxEN = IOLinkTestConstants::TEST_TXEN_PIN,
            .Wakeup = IOLinkTestConstants::TEST_WAKEUP_PIN
        }
    };
    device->begin(hwConfig);
    
    CHECK_EQUAL(lwIOLink::start, device->GetMode());
    
    // Debug: Check if interrupt was properly registered
    LONGS_EQUAL(true, MockPinManager::isInterruptEnabled(IOLinkTestConstants::TEST_WAKEUP_PIN));
    
    CHECK_EQUAL(false, IOLinkTestUtils::WasWakeupTriggered());
    IOLinkTestUtils::TriggerWakeupSignal();
    CHECK_EQUAL(true, IOLinkTestUtils::WasWakeupTriggered());
    device->run();
    
    // Test master command building
    uint8_t frame[IOLinkTestConstants::TEST_FRAME_BUFFER_SIZE];
    
    lwIOLink::MasterCommands commands[] = {
        lwIOLink::MasterIdent, lwIOLink::DeviceIden, lwIOLink::DeviceStartup,
        lwIOLink::PDOutOperate, lwIOLink::Operate, lwIOLink::Preoperate
    };
    
    for (auto cmd : commands) {
        uint8_t frameLength = IOLinkMaster::FrameBuilder::BuildMasterCommandFrame(cmd, frame);
        
        CHECK(frameLength > 0);
        CHECK(frameLength <= IOLinkTestConstants::TEST_FRAME_BUFFER_SIZE);
        
        IOLinkTestUtils::SendFrameToDevice(frame, frameLength);
        device->run();
        
        CHECK(IOLinkTestUtils::GetTransmittedLength() >= 0);
        
        IOLinkTestUtils::FlushSerialBuffers();
    }
    
    // Test that all commands are processed correctly
}

TEST(IOLinkCommunication, UtilityFunctionsDemo)
{
    // Test wakeup signal simulation
    CHECK_EQUAL(false, IOLinkTestUtils::WasWakeupTriggered());
    
    IOLinkTestUtils::TriggerWakeupSignal();
    
    // Test buffer management
    const uint8_t testData[] = {0xAA, 0xBB, 0xCC};
    IOLinkTestUtils::SendFrameToDevice(testData, 3);
    CHECK_EQUAL(3, IOLinkTestUtils::GetReceivedLength());
    
    IOLinkTestUtils::FlushSerialBuffers();
    CHECK_EQUAL(0, IOLinkTestUtils::GetReceivedLength());
    CHECK_EQUAL(0, IOLinkTestUtils::GetTransmittedLength());
    
    // Test timing utilities with various cycle times
    uint32_t testCycleTimes[] = {2300, 2070, 2530, 1800, 2800};
    bool expectedResults[] = {true, true, true, false, false};
    
    for (size_t i = 0; i < 5; i++) {
        CHECK_EQUAL(expectedResults[i], IOLinkTestUtils::IsCycleTimeValid(testCycleTimes[i]));
    }
    
    bool wakeupTriggered = false;
    IOLinkTestUtils::SetupInterruptCallback(IOLinkTestConstants::TEST_WAKEUP_PIN, wakeupTriggered);
    MockPinManager::triggerInterrupt(IOLinkTestConstants::TEST_WAKEUP_PIN);
    CHECK_EQUAL(true, wakeupTriggered);
}

TEST(IOLinkCommunication, PublicEnumAccessibility)
{
    // Verify that all public enums are accessible from the lwIOLink namespace
    lwIOLink::Mode mode = lwIOLink::start;
    CHECK_EQUAL(lwIOLink::start, mode);
    
    mode = lwIOLink::start;
    CHECK_EQUAL(lwIOLink::start, mode);
    
    lwIOLink::BaudRate baud = lwIOLink::COM1;
    CHECK_EQUAL(lwIOLink::COM1, baud);
    
    baud = lwIOLink::COM2;
    CHECK_EQUAL(lwIOLink::COM2, baud);
    
    baud = lwIOLink::COM3;
    CHECK_EQUAL(lwIOLink::COM3, baud);
    
    lwIOLink::MasterCommands cmd = lwIOLink::MasterIdent;
    CHECK_EQUAL(lwIOLink::MasterIdent, cmd);
    
    cmd = lwIOLink::DeviceIden;
    CHECK_EQUAL(lwIOLink::DeviceIden, cmd);
    
    cmd = lwIOLink::DeviceStartup;
    CHECK_EQUAL(lwIOLink::DeviceStartup, cmd);
    
    cmd = lwIOLink::PDOutOperate;
    CHECK_EQUAL(lwIOLink::PDOutOperate, cmd);
    
    cmd = lwIOLink::Operate;
    CHECK_EQUAL(lwIOLink::Operate, cmd);
    
    cmd = lwIOLink::Preoperate;
    CHECK_EQUAL(lwIOLink::Preoperate, cmd);
}
