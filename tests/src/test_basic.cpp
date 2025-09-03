// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "CppUTest/TestHarness.h"
#include "lwIOLink.hpp"

TEST_GROUP(BasicTests)
{
    void setup()
    {
        // Setup code before each test
    }

    void teardown()
    {
        // Cleanup code after each test
    }
};

TEST(BasicTests, SanityCheck)
{
    // Basic sanity test to verify CppUTest is working
    CHECK_EQUAL(1, 1);
    CHECK_TRUE(true);
    CHECK_FALSE(false);
}

TEST(BasicTests, DeviceCreation)
{
    // Test basic device creation
    lwIOLink::Device device(2, 2, 1000);
    
    // Check initial mode
    CHECK_EQUAL(lwIOLink::start, device.GetMode());
}

TEST(BasicTests, CycleTimeEncoding)
{
    // Test cycle time encoding/decoding functionality using Utils functions
    
    // Test with minimum cycle time (400 us should be adjusted to 400 us)
    uint8_t encoded = lwIOLink::Utils::EncodeCycleTime(400);
    uint32_t decoded = lwIOLink::Utils::DecodeCycleTime(encoded);
    
    // Should be able to encode and decode consistently
    CHECK_TRUE(decoded >= 400);
    CHECK_TRUE(decoded <= 132800); // Maximum cycle time
}

TEST(BasicTests, PDOperations)
{
    // Test Process Data operations
    lwIOLink::Device device(2, 2, 1000);
    
    uint8_t test_data[] = {0xAB, 0xCD};
    uint8_t buffer[2];
    lwIOLink::PDStatus status;
    
    // Should return false when not in operate mode
    CHECK_FALSE(device.SetPDIn(test_data, 2));
    CHECK_FALSE(device.GetPDOut(buffer, &status));
    CHECK_FALSE(device.SetPDInStatus(lwIOLink::Valid));
}
