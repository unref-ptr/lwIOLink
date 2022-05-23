/*
	Copyright (C) 2022 unref-ptr
    This file is part of lwIOLink.
    lwIOLink is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    lwIOLink is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with lwIOLink.  If not, see<https://www.gnu.org/licenses/>.

    Contact information:
   <unref-ptr@protonmail.com>
*/
#include "lwIOLink.hpp"

#ifdef ARDUINO_ARCH_ESP32
#include "driver/uart.h"
#include "soc/uart_reg.h"
#endif	//ARDUINO_ARCH_ESP32

using namespace lwIOLink;

//Checksum constants (Figure A.3)
static constexpr uint8_t ck8_seed = 0x52;
static constexpr uint8_t status_bit_offset = 0x06;
static constexpr uint8_t eventFlag_bit_offset = 0x07;
static constexpr uint8_t ckt_offset = 0x01;
//Figure A.2
static uint8_t mseq_ckt_mask = 0xC0;
static uint8_t ck6_mask = 0x3F;
//Time encoding constants (Figure B.2)
static constexpr uint8_t timebase_bitoffset = 0x06;
static constexpr uint8_t multiplier_mask = 0x3F;
//MC Constants (Figure A.1)
static constexpr uint8_t MC_ChMask = 0x60;
static constexpr uint8_t MC_AddrMask = 0x1F;
static constexpr uint8_t MC_ChBitOffset = 5;

uint32_t constexpr Device::TimeBaseLUT[TotalTimeEncodings];
uint32_t constexpr Device::TimeOffsetLUT[TotalTimeEncodings];
static volatile bool wakeup_signal = false;


static void WakeupIRQ()
{
    wakeup_signal = true;
}

void __attribute__((weak)) Device::OnEventsProcessed() 
{
    
}

uint8_t Device::GetChecksum(const uint8_t *data,
                            uint8_t length) const
{
    uint8_t ck8 = ck8_seed;
    for (uint8_t i = 0; i < length; i++)
    {
        ck8 ^= *data++;
    }
   	//Section A.1.6
    const uint8_t bit5 = ((ck8 >> 7) & 1U) ^ ((ck8 >> 5) & 1U) ^ ((ck8 >> 3) & 1U) ^ ((ck8 >> 1) & 1U);
    const uint8_t bit4 = ((ck8 >> 6) & 1U) ^ ((ck8 >> 4) & 1U) ^ ((ck8 >> 2) & 1U) ^ (ck8 & 1U);
    const uint8_t bit3 = ((ck8 >> 7) & 1U) ^ ((ck8 >> 6) & 1U);
    const uint8_t bit2 = ((ck8 >> 5) & 1U) ^ ((ck8 >> 4) & 1U);
    const uint8_t bit1 = ((ck8 >> 3) & 1U) ^ ((ck8 >> 2) & 1U);
    const uint8_t bit0 = ((ck8 >> 1) & 1U) ^ ((ck8 & 1U));
    const uint8_t ck6 =     bit5 << 5 |
                            bit4 << 4 |
                            bit3 << 3 |
                            bit2 << 2 |
                            bit1 << 1 |
                            bit0;
    return ck6;
}

//Encode PD according to B.1.6
uint8_t EncodePD(uint8_t size_bytes)
{
    uint8_t Byte;
    uint8_t Len;
   	// Table B.6
    if (size_bytes <= 2)
    {
        Byte = 0;
        Len = size_bytes * 8;
    }
    else
    {
        Byte = 1;
        Len = size_bytes - 1;
    }

    return Byte << 7 | (Len & 0x1F);
}

uint8_t Device::GetMseqCap() const
{
    uint8_t PreopCode;
    uint8_t OpCode;
   	//Table A.8
    if (ODSize.Preop == 1)
    {
        PreopCode = 0;
    }
    else if (ODSize.Preop == 2)
    {
        PreopCode = 1;
    }
    else if (ODSize.Preop == 8)
    {
        PreopCode = 2;
    }
    else
    {
        PreopCode = 3;
    }
   	//Table A.10
    if (ODSize.Op == 1)
    {
        if ((Pd.In.Size > 0 && Pd.Out.Size >= 3) ||
            (Pd.In.Size >= 3 && Pd.Out.Size > 0)
       )
        {
            OpCode = 4;
        }
        else
        {
            OpCode = 0;
        }
    }
    else if (ODSize.Op == 2)
    {
        if (Pd.In.Size == 0 && Pd.Out.Size == 0)
        {
            OpCode = 1;
        }
        else
        {
            OpCode = 5;
        }
    }
    else if (ODSize.Op == 8)
    {
        OpCode = 6;
    }
    else
    {
        OpCode = 7;
    }
    return PreopCode << 4 | OpCode << 1 | static_cast<uint8_t> (ISDUSupported);
}

uint32_t Device::DecodeCycleTime(uint8_t encoded_time) const
{
    const uint8_t timebase_code = encoded_time >> timebase_bitoffset;
    const uint8_t multiplier = encoded_time & multiplier_mask;
    return TimeOffsetLUT[timebase_code] + (multiplier *TimeBaseLUT[timebase_code]);
}

uint8_t Device::EncodeCycleTime(uint32_t cycleTime_us) const
{
    uint8_t timebase_code;
    uint8_t multiplier;
    const unsigned max_multiplier = 63;
    if (cycleTime_us < 400)
    {
        cycleTime_us = 400;
    }
    else if (cycleTime_us > 6300 && cycleTime_us < 6400)
    {
        cycleTime_us = 6300;
    }
    else if (cycleTime_us > 31600 && cycleTime_us < 32000)
    {
        cycleTime_us = 31600;
    }
    else if (cycleTime_us > 132800)
    {
        cycleTime_us = 132800;
    }

    if (cycleTime_us <= 6300)
    {
        timebase_code = 0;
    }
    else if (cycleTime_us <= 31600)
    {
        timebase_code = 1;
    }
    else
    {
        timebase_code = 2;
    }
    for (multiplier = 0; multiplier <= max_multiplier; multiplier++)
    {
        uint32_t cycletime_match = TimeOffsetLUT[timebase_code] + (multiplier *TimeBaseLUT[timebase_code]);
        if (cycletime_match >= cycleTime_us)
        {
            break;
        }
    }
    return (timebase_code << timebase_bitoffset | multiplier);
}

Device::Device(uint8_t PDIn, uint8_t PDOut, uint32_t min_cycletime)
{
    EventMemory[EventStatusCodeAddr] = Event::StatusCodeDefault;
    Pd.Out.Size = PDOut;
    Pd.In.Size = PDIn;
    /*Populate Direct Parameter Page 1 */
    ParameterPage1[DP1_Param::MinCycleTime] = EncodeCycleTime(min_cycletime);
    ParameterPage1[DP1_Param::MSeqCap] = GetMseqCap();
    ParameterPage1[DP1_Param::ProcessDataIn] = EncodePD(PDIn);
    ParameterPage1[DP1_Param::ProcessDataOut] = EncodePD(PDOut);
    ParameterPage1[DP1_Param::RevisionID] = 0x11;
    ParameterPage1[DP1_Param::VID1] = 0xAB;
    ParameterPage1[DP1_Param::VID2] = 0xCD;
    ParameterPage1[DP1_Param::DID1] = 0x12;
    ParameterPage1[DP1_Param::DID2] = 0x34;
    ParameterPage1[DP1_Param::DID3] = 0x56;
    ParameterPage1[DP1_Param::FID1] = 0x00;
    ParameterPage1[DP1_Param::FID2] = 0x00;
    CurrentCycleTime = DecodeCycleTime(ParameterPage1[DP1_Param::MinCycleTime]);
}

bool Device::GetPDOut(uint8_t *buffer, PDStatus *pStatus) const
{
    memcpy(buffer, Pd.Out.Data, Pd.Out.Size);
    if (deviceMode == operate)
    {
        *pStatus = status.PDOut;
        return true;
    }
    else
    {
        return false;
    }
}

        
Device::EventResult Device::SetEvent(Event::POD newEvent)
{
   EventResult result = EventResult::EventOK;
   do
   {
        if (ReadingEventMemory == true)
        {
            result = EventResult::ProcessingEvents;
            break;
        }
        if (TotalEvents < MaxEvents )
        {
           const uint8_t memory_offset = Event::SizeStatusCode + (TotalEvents * Event::SizeRawEvent);
           const uint8_t QualifierOffset = memory_offset;
           const uint8_t EventCodeMSB = memory_offset + 1U;
           const uint8_t EventCodeLSB = memory_offset + 2U;
           EventMemory[QualifierOffset] = newEvent.EventQualifier;
           EventMemory[EventCodeMSB] = static_cast<uint8_t>(newEvent.EventCode >> 8U);
           EventMemory[EventCodeLSB] = static_cast<uint8_t>(newEvent.EventCode & 0xFF);

           EventMemory[EventStatusCodeAddr] |= 1UL << TotalEvents;
           
           TotalEvents++;
        }
        else
        {
            result = EventResult::EventMemoryFull;
            break;
        }
      
   } while(0);   
   return result;
   
}

lwIOLink::Mode Device::GetMode() const
{
    return deviceMode;
}

bool Device::SetPDInStatus(PDStatus pd_status)
{
    if (deviceMode == operate)
    {
        status.PDIn = pd_status;
        return true;
    }
    else
    {
        return false;
    }
}

bool Device::SetPDIn(uint8_t *pData, uint8_t len)
{
    if (deviceMode == operate)
    {
        if (len <= Pd.In.Size)
        {
            memcpy(Pd.In.Data, pData, len);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void Device::ProcessMessage()
{
    uint8_t MasterOD_offset = MCSize + ChecksumSize;
    ParseMC();
    memset(ODBuffer, 0, sizeof(ODBuffer));	//Clear OD
    if (deviceMode == operate)
    {
        MasterOD_offset += Pd.Out.Size;
    }
    switch (message.channel)
    {
        case Page:
            if (MasterAccess == MCAccess::Read)
            {
                ODBuffer[0] = ParameterPage1[message.addr];
            }
            else
            {
                auto write_param = static_cast<DP1_Param> (message.addr);
                const uint8_t ODWrite = rxBuffer[MasterOD_offset];
                if (write_param == DP1_Param::MasterCommand)
                {
                    Cmd = static_cast<MasterCommands> (ODWrite);
                    NewCmd = true;
                }
                if (write_param == DP1_Param::MasterCycleTime)
                {
                    CurrentCycleTime = DecodeCycleTime(ODWrite);
                }
            }
            break;
        case ISDU: //Not implemented
            if (MasterAccess == MCAccess::Read)
            {
                memset(ODBuffer, 0x69, sizeof(ODBuffer));
            }
            break;
        case Diagnosis:
            if ( MasterAccess == MCAccess::Read)
            {               
                if( TotalEvents > 0 )
                {
                    if ( ReadingEventMemory != true)
                    {
                        ReadingEventMemory = true;
                    }
                }
                if( message.addr < sizeof(EventMemory) ) 
                {
                    const uint8_t read_bytes = sizeof(EventMemory) - message.addr;
                    uint8_t od_size;
                    if (deviceMode == start)
                    {
                        od_size = ODSize.Startup;
                    }
                    else if (deviceMode == preoperate)
                    {
                        od_size = ODSize.Preop;
                    }
                    else
                    {
                        od_size = ODSize.Op;
                    }
                    uint8_t copy_size;
                    if( read_bytes < od_size)
                    {
                        copy_size = read_bytes;
                    }
                    else
                    {
                        copy_size = od_size; 
                    }
                    memcpy(ODBuffer,&EventMemory[message.addr],copy_size);
                }
            }
            else
            {
                if ( message.addr == EventStatusCodeAddr)
                {
                    ReadingEventMemory = false;
                    TotalEvents = 0;
                    EventMemory[EventStatusCodeAddr] = Event::StatusCodeDefault;
                    EventsProcessed = true;
                }
            }
            break;
    }
}

uint8_t Device::SetResponse()
{
    uint8_t tx_size = ChecksumSize;
    uint8_t od_size = 0;
    memset(txBuffer, 0, sizeof(txBuffer));
    switch (deviceMode)
    {
        case start:
            od_size = ODSize.Startup;
            break;
        case preoperate:
            od_size = ODSize.Preop;
            break;
        case operate:
            od_size = ODSize.Op;
            tx_size += Pd.In.Size;
            uint8_t pd_offset = 0;
            if (MasterAccess == MCAccess::Read)
            {
                pd_offset += ODSize.Op;
            }
            memcpy(&txBuffer[pd_offset], Pd.In.Data, Pd.In.Size);
            break;
    }
    if (MasterAccess == MCAccess::Read)
    {
        memcpy(txBuffer, ODBuffer, od_size);
        tx_size += od_size;
    }
    const uint8_t checksum_offset = tx_size - 1;
    bool eventFlag = false;
    if ( TotalEvents > 0)
    {
        eventFlag = true;
    }
    const uint8_t status_encoded =  static_cast<uint8_t>(status.PDIn << status_bit_offset)
                                    | static_cast<uint8_t>(eventFlag<<eventFlag_bit_offset);
    txBuffer[checksum_offset] = status_encoded;
    uint8_t ck6 = GetChecksum(txBuffer, tx_size);
    txBuffer[checksum_offset] |= ck6;
    return tx_size;
}


inline void Device::ParseMC()
{
    message.channel = (rxBuffer[MCOffset] & MC_ChMask) >> MC_ChBitOffset;
    message.addr = rxBuffer[MCOffset] & MC_AddrMask;
}

void Device::begin(const HWConfig config)
{
    TxEn = config.Pin.TxEN;
    WuPin = config.Pin.Wakeup;
    SerialPort = &config.SerialPort;
#ifdef ARDUINO_SAM_DUE
    static_cast<UARTClass*>(SerialPort)->begin(static_cast<uint32_t> (config.Baud), SERIAL_8E1);
#elif defined(ARDUINO_ARCH_ESP32)
    static_cast<HardwareSerial*>(SerialPort)->begin(static_cast<uint32_t> (config.Baud), SERIAL_8E1, config.Pin.Rx, config.Pin.Tx);
    uint8_t uart_num;
    if (SerialPort == &Serial)
    {
        uart_num = 0;
    }
    else if (SerialPort == &Serial1)
    {
        uart_num = 1;
    }
    else
    {
        uart_num = 2;
    }
    uart_intr_config_t uart_intr;
    uart_intr.intr_enable_mask =    UART_RXFIFO_FULL_INT_ENA_M |
                                    UART_RXFIFO_TOUT_INT_ENA_M |
                                    UART_FRM_ERR_INT_ENA_M |
                                    UART_RXFIFO_OVF_INT_ENA_M |
                                    UART_BRK_DET_INT_ENA_M |
                                    UART_PARITY_ERR_INT_ENA_M;
    uart_intr.rxfifo_full_thresh = 1;
    uart_intr.rx_timeout_thresh = 10;
    uart_intr.txfifo_empty_intr_thresh = 10;
    uart_intr_config(uart_num, &uart_intr);
#else
#ifdef ARDUINO_RASPBERRY_PI_PICO
    static_cast<SerialUART*>(SerialPort)->setRX(config.Pin.Rx);
    static_cast<SerialUART*>(SerialPort)->setTX(config.Pin.Tx);
    static_cast<SerialUART*>(SerialPort)->setPollingMode(true);
#endif //ARDUINO_RASPBERRY_PI_PICO
    static_cast<HardwareSerial*>(SerialPort)->begin(static_cast<uint32_t> (config.Baud), SERIAL_8E1);
#endif
    initTransciever(config.WakeupMode);
}

inline uint8_t Device::GetMasterTXSize()
{
    uint8_t od_size = 0;
    uint8_t pd_size = 0;
    if (deviceMode == start)
    {
        if (MasterAccess == MCAccess::Write)
        {
            od_size = ODSize.Startup;
        }
    }
    else if (deviceMode == preoperate)
    {
        if (MasterAccess == MCAccess::Write)
        {
            od_size = ODSize.Preop;
        }
    }
    else	//Operate mod
    {
        if (MasterAccess == MCAccess::Write)
        {
            od_size = ODSize.Op;
        }
        pd_size = Pd.Out.Size;
    }
    return MasterMetadataOffset + od_size + pd_size;
}

void Device::SaveMasterFrame(const uint8_t rx_byte)
{
    rxBuffer[rxCnt] = rx_byte;
    if (rxCnt == MCOffset)
    {
        MasterAccess = static_cast<MCAccess> (rxBuffer[MCOffset] >> 7);	//Figure A.1
        ExpectedRXCnt = GetMasterTXSize();
    }
    if (++rxCnt == ExpectedRXCnt)
    {
        const uint8_t master_checksum = rxBuffer[ckt_offset] & ck6_mask;
        rxBuffer[ckt_offset] &= mseq_ckt_mask; //Remove checksum
        const uint8_t calculated_checksum = GetChecksum(rxBuffer,rxCnt); //Calculate the checksum
        if ( master_checksum == calculated_checksum)
        {
            NewMasterMsg = true;
        }
        else
        {
            rxCnt = 0;
            ExpectedRXCnt = 0xFF;
        }
    }
}

void Device::ResetRX()
{
    rxCnt = 0;
    ExpectedRXCnt = 0xFF;
    while (SerialPort->available() > 0)
    {
        SerialPort->read();
    }
}

void Device::initTransciever(int wakeup_mode) const
{
    pinMode(TxEn, OUTPUT);
    pinMode(WuPin, INPUT_PULLUP);
    digitalWrite(TxEn, HIGH);
#ifdef ARDUINO_RASPBERRY_PI_PICO
    attachInterrupt(digitalPinToInterrupt(WuPin), WakeupIRQ, static_cast<PinStatus>(wakeup_mode));
#else
    attachInterrupt(digitalPinToInterrupt(WuPin), WakeupIRQ, wakeup_mode);
#endif
}

void Device::DeviceRsp(uint8_t *data, uint8_t len)
{
    digitalWrite(TxEn, HIGH);
    for (uint8_t i = 0; i < len; i++)
    {
        SerialPort->write(*(data + i));
    }
    SerialPort->flush();	//Wait until transmission is finished
    digitalWrite(TxEn, LOW);
    ResetRX();
}

void Device::run()
{
    switch (deviceState)
    {
        case wait_wake:
            if (wakeup_signal)
            {
                wakeup_signal = false;
                deviceState = wait_valid_frame;
                digitalWrite(TxEn, LOW);
            }
            break;
        case wait_valid_frame:
           	//First valid frame 0xA2 = MC (Read Page Channel, Min Cycle Time)
            if (SerialPort->available() > 0)
            {
                const uint8_t rx_byte = SerialPort->read();
                if (rx_byte == 0xA2)
                {
                    deviceState = run_mode;
                    SaveMasterFrame(rx_byte);
                }
            }
            break;
        case run_mode:
            unsigned long current_time = micros();
            if (SerialPort->available() > 0)
            {
                const uint8_t rx_byte = SerialPort->read();
                SaveMasterFrame(rx_byte);
            }
            if (deviceMode == operate &&
                (current_time - LastMessage) > CurrentCycleTime)
            {
                deviceMode = start;
                deviceState = wait_wake;
                ResetRX();
                digitalWrite(TxEn, HIGH);
            }
            else if (NewMasterMsg)
            {
                NewMasterMsg = false;
                ProcessMessage();
                const uint8_t tx_size = SetResponse();
                DeviceRsp(txBuffer, tx_size);
                if (EventsProcessed == true)
                {
                    EventsProcessed = false;
                    OnEventsProcessed();
                }
                LastMessage = micros();
                if (deviceMode == operate)
                {
                    OnNewCycle();
                }
            }
            break;
    }
    if (NewCmd)
    {
        NewCmd = false;
        switch (Cmd)
        {
            case MasterCommands::Operate:
                LastMessage = micros();
                deviceMode = operate;
                status.PDOut = Invalid;
                break;
            case MasterCommands::PDOutOperate:
                status.PDOut = Valid;
                break;
            case MasterCommands::Preoperate:
                deviceMode = preoperate;
                break;
        }
    }
}
