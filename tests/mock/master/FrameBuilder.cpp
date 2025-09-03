// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "FrameBuilder.h"

namespace IOLinkMaster {

uint8_t FrameBuilder::BuildReadDP1Frame(lwIOLink::DP1_Param address, uint8_t* frame_buffer) {
    // Frame structure for reading Direct Parameter Page 1:
    // [MC] [CKT]
    // MC = Read access, Page channel, address
    // CKT = Type 0, checksum

    frame_buffer[0] = BuildMC(lwIOLink::MCAccess::Read, lwIOLink::Channel::Page, static_cast<uint8_t>(address));

    // Calculate checksum (only MC byte for now)
    uint8_t checksum = CalculateChecksum(frame_buffer, 1);

    frame_buffer[1] = BuildCKT(lwIOLink::MSeqType::Type0, checksum);

    return 2; // Frame length
}

uint8_t FrameBuilder::BuildWriteDP1Frame(lwIOLink::DP1_Param address, uint8_t data, uint8_t* frame_buffer) {
    // Frame structure for writing Direct Parameter Page 1:
    // [MC] [CKT] [DATA]

    frame_buffer[0] = BuildMC(lwIOLink::MCAccess::Write, lwIOLink::Channel::Page, static_cast<uint8_t>(address));
    frame_buffer[2] = data; // Data comes after CKT
    
    // Calculate checksum for MC + DATA (CKT will be inserted)
    uint8_t temp_frame[2] = {frame_buffer[0], data};
    uint8_t checksum = CalculateChecksum(temp_frame, 2);

    frame_buffer[1] = BuildCKT(lwIOLink::MSeqType::Type0, checksum);

    return 3; // Frame length
}

uint8_t FrameBuilder::BuildMasterCommandFrame(lwIOLink::MasterCommands command, uint8_t* frame_buffer) {
    // Frame structure for Master Command (write to DP1 address 0x00):
    // [MC] [CKT] [COMMAND]

    frame_buffer[0] = BuildMC(lwIOLink::MCAccess::Write, lwIOLink::Channel::Page, static_cast<uint8_t>(lwIOLink::MasterCommand));
    frame_buffer[2] = static_cast<uint8_t>(command);
    
    // Calculate checksum for MC + COMMAND
    uint8_t temp_frame[2] = {frame_buffer[0], static_cast<uint8_t>(command)};
    uint8_t checksum = CalculateChecksum(temp_frame, 2);

    frame_buffer[1] = BuildCKT(lwIOLink::MSeqType::Type0, checksum);

    return 3; // Frame length
}

uint8_t FrameBuilder::BuildMC(lwIOLink::MCAccess rw, lwIOLink::Channel channel, uint8_t address) {
    MCByte mc(rw, channel, address);
    return mc.toByte();
}

uint8_t FrameBuilder::BuildCKT(lwIOLink::MSeqType type, uint8_t checksum) {
    CKTByte ckt(type, checksum);
    return ckt.toByte();
}

uint8_t FrameBuilder::CalculateChecksum(const uint8_t* data, uint8_t length) {
    // Implementation matching the Device::GetChecksum method
    uint8_t ck8 = CK8_SEED;
    for (uint8_t i = 0; i < length; i++) {
        ck8 ^= data[i];
    }
    
    // Section A.1.6 - Convert CK8 to CK6
    const uint8_t bit5 = ((ck8 >> 7) & 1U) ^ ((ck8 >> 5) & 1U) ^ ((ck8 >> 3) & 1U) ^ ((ck8 >> 1) & 1U);
    const uint8_t bit4 = ((ck8 >> 6) & 1U) ^ ((ck8 >> 4) & 1U) ^ ((ck8 >> 2) & 1U) ^ (ck8 & 1U);
    const uint8_t bit3 = ((ck8 >> 7) & 1U) ^ ((ck8 >> 6) & 1U);
    const uint8_t bit2 = ((ck8 >> 5) & 1U) ^ ((ck8 >> 4) & 1U);
    const uint8_t bit1 = ((ck8 >> 3) & 1U) ^ ((ck8 >> 2) & 1U);
    const uint8_t bit0 = ((ck8 >> 1) & 1U) ^ ((ck8 & 1U));
    
    const uint8_t ck6 = bit5 << 5 |
                        bit4 << 4 |
                        bit3 << 3 |
                        bit2 << 2 |
                        bit1 << 1 |
                        bit0;
    return ck6;
}

} // namespace IOLinkMaster
