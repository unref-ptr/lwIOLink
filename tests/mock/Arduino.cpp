// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "Arduino.h"

// Define static members for MockPinManager
uint8_t MockPinManager::pinModes[MockPinManager::MAX_PINS] = {0};
uint8_t MockPinManager::pinValues[MockPinManager::MAX_PINS] = {0};
unsigned long MockPinManager::startTime = 0;
bool MockPinManager::timeInitialized = false;
unsigned long MockPinManager::currentTime = 0;
InterruptCallback MockPinManager::interruptCallbacks[MockPinManager::MAX_PINS] = {nullptr};
bool MockPinManager::interruptEnabled[MockPinManager::MAX_PINS] = {false};
bool MockPinManager::interruptTriggered[MockPinManager::MAX_PINS] = {false};

// Global mock serial instance
MockSerial Serial;
