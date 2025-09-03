// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#pragma once

// Master frame utilities for testing IO-Link communication
#include <stdint.h>
#include "lwIOLink.hpp" // Include main header to use existing enums

namespace IOLinkMaster {

// Packed structure for Master Control (MC) byte
struct __attribute__((packed)) MCByte {
    uint8_t address : 5;    // Bits 0-4: Address (5 bits)
    uint8_t channel : 2;    // Bits 5-6: Channel (2 bits)  
    uint8_t rw : 1;         // Bit 7: Read/Write access (1 bit)

    MCByte(lwIOLink::MCAccess access, lwIOLink::Channel ch, uint8_t addr) 
        : address(addr), channel(ch), rw(static_cast<uint8_t>(access)) {}
    
    uint8_t toByte() const {
        return *reinterpret_cast<const uint8_t*>(this);
    }
};

// Packed structure for Checksum/M-sequence Type (CKT) byte  
struct __attribute__((packed)) CKTByte {
    uint8_t checksum : 6;   // Bits 0-5: Checksum (6 bits)
    uint8_t type : 2;       // Bits 6-7: M-sequence type (2 bits)

    CKTByte(lwIOLink::MSeqType seq_type, uint8_t ck) 
        : checksum(ck & 0x3F), type(static_cast<uint8_t>(seq_type)) {}

    uint8_t toByte() const {
        return *reinterpret_cast<const uint8_t*>(this);
    }
};

class FrameBuilder {
public:
    // Build a complete master frame for reading Direct Parameter Page 1
    // Uses lwIOLink::DP1_Param enum for addresses
    static uint8_t BuildReadDP1Frame(lwIOLink::DP1_Param address, uint8_t* frame_buffer);
    
    // Build a complete master frame for writing Direct Parameter Page 1  
    static uint8_t BuildWriteDP1Frame(lwIOLink::DP1_Param address, uint8_t data, uint8_t* frame_buffer);
    
    // Build frame for Master Command (address 0x00) with specific command
    // Uses lwIOLink::MasterCommands enum  
    static uint8_t BuildMasterCommandFrame(lwIOLink::MasterCommands command, uint8_t* frame_buffer);
    
    // Build Master Control (MC) byte using packed struct
    static uint8_t BuildMC(lwIOLink::MCAccess rw, lwIOLink::Channel channel, uint8_t address);
    
    // Build Checksum/M-sequence Type (CKT) byte using packed struct
    static uint8_t BuildCKT(lwIOLink::MSeqType type, uint8_t checksum);
    
    // Calculate 6-bit checksum for a frame
    static uint8_t CalculateChecksum(const uint8_t* data, uint8_t length);
    
private:
    static constexpr uint8_t CK8_SEED = 0x52;
};

// Compile-time assertions to ensure struct sizes are correct
static_assert(sizeof(MCByte) == 1, "MCByte must be exactly 1 byte");
static_assert(sizeof(CKTByte) == 1, "CKTByte must be exactly 1 byte");

} // namespace IOLinkMaster
