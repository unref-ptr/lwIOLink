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
    along with lwIOLink.  If not, see <https://www.gnu.org/licenses/>.
    
    Contact information:
    <unref-ptr@protonmail.com>
*/

#include "lwIOLink.hpp"

uint32_t constexpr lwIOLink::TimeBaseLUT[TotalTimeEncodings];
uint32_t constexpr lwIOLink::TimeOffsetLUT[TotalTimeEncodings];
static volatile bool wakeup_signal = false;

static void WakeupIRQ()
{
  wakeup_signal = true;
}

uint8_t lwIOLink::GetChecksum(uint8_t *data,
                                     uint8_t length,
                                     PDStatus status)
{
  uint8_t ck8 = 0x52; //checksum Seed
  ck8 ^=  (status << 6);
  for (int i = 0; i < length - 1; i++)
  {
    ck8 ^= *data++;
  }
  //Section A.1.6
  bool bit5 = ((ck8 >> 7)  & 1U) ^ ((ck8 >> 5)  & 1U) ^ ((ck8 >> 3)  & 1U) ^ ((ck8 >> 1)  & 1U);
  bool bit4 = ((ck8 >> 6)  & 1U) ^ ((ck8 >> 4)  & 1U) ^ ((ck8 >> 2)  & 1U)  ^ (ck8 & 1U);
  bool bit3 = ((ck8 >> 7)  & 1U) ^ ((ck8 >> 6)  & 1U);
  bool bit2 = ((ck8 >> 5)  & 1U) ^ ((ck8 >> 4)  & 1U);
  bool bit1 = ((ck8 >> 3)  & 1U) ^ ((ck8 >> 2)  & 1U);
  bool bit0 = ((ck8 >> 1)  & 1U) ^ ((ck8 & 1U));
  uint8_t ck6 = bit5 << 5 |
                bit4 << 4 |
                bit3 << 3 |
                bit2 << 2 |
                bit1 << 1 |
                bit0;
  return (status << 6) | ck6;
}

//Encode PD according to B.1.6
inline uint8_t EncodePD(uint8_t size_bytes)
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

  return Byte << 7 | Len & 0x1F;
}

uint8_t lwIOLink::GetMseqCap()
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
    if ( (Pd.In.Size > 0 && Pd.Out.Size >= 3)
         || (Pd.In.Size >= 3 && Pd.Out.Size > 0)
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
  return PreopCode << 4 | OpCode << 1 | ISDUSupported;
}


uint32_t lwIOLink::DecodeCycleTime(uint8_t encoded_time)
{
  uint8_t timebase_code = encoded_time>>6;
  uint8_t multiplier = encoded_time & 0x3F;
  return TimeOffsetLUT[timebase_code] + (multiplier * TimeBaseLUT[timebase_code]);
}

uint8_t lwIOLink::EncodeCycleTime(uint32_t cycleTime_us)
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

  if (cycleTime_us >= 400 && cycleTime_us <= 6300)
  {
    timebase_code = 0;
  }
  else if (cycleTime_us >= 6400 && cycleTime_us <= 31600)
  {
    timebase_code = 1;
  }
  else if (cycleTime_us >= 32000 && cycleTime_us <= 132800)
  {
    timebase_code = 2;
  }
  for (multiplier = 0; multiplier <= max_multiplier; multiplier++)
  {
    uint32_t cycletime_match = TimeOffsetLUT[timebase_code] + (multiplier * TimeBaseLUT[timebase_code]);
    if (cycletime_match >= cycleTime_us)
    {
      break;
    }
  }
  return (timebase_code << 6 | multiplier);
}

lwIOLink::lwIOLink(uint8_t PDIn, uint8_t PDOut, uint32_t min_cycletime)
{
  memset(Pd.Out.Data, 0, sizeof(Pd.Out.Data));
  memset(Pd.In.Data, 0, sizeof(Pd.Out.Data));
  Pd.Out.Size = PDOut;
  Pd.In.Size = PDIn;
  /* Populate Direct Parameter Page 1 */
  ParameterPage1[DP1_Param::MinCycleTime] = EncodeCycleTime(min_cycletime);
  ParameterPage1[DP1_Param::MSeqCap] =   GetMseqCap();
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

bool lwIOLink::GetPDOut(uint8_t * buffer, PDStatus * pStatus)
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

lwIOLink::Mode lwIOLink::GetMode()
{
  return deviceMode;
}

bool lwIOLink::SetPDInStatus(PDStatus pd_status)
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

bool lwIOLink::SetPDIn(uint8_t * pData, uint8_t len)
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

void lwIOLink::ProcessMessage()
{
  uint8_t MasterOD_offset = MCSize + ChecksumSize;
  ParseMC();
  memset(ODBuffer, 0, sizeof(ODBuffer)); //Clear OD
  if ( deviceMode == operate)
  {
    MasterOD_offset += Pd.Out.Size;
  }
  switch (message.channel)
  {
    case PAGE:
      if (MasterAccess == MCAccess::Read)
      {
        ODBuffer[0] = ParameterPage1[message.addr];
      }
      else
      {
        DP1_Param write_param = static_cast<DP1_Param>(message.addr);
        uint8_t ODWrite = rxBuffer[MasterOD_offset];
        if (write_param == DP1_Param::MasterCommand)
        {
          Cmd = static_cast<MasterCommands>(ODWrite);
          NewCmd = true;
        }
        if(write_param == DP1_Param::MasterCycleTime)
        {
           CurrentCycleTime = DecodeCycleTime(ODWrite);
        }
      }
      break;
    //Not implemented
    case ISDU:
    case DIAGNOSIS:
      if (MasterAccess == MCAccess::Read)
      {
        memset(ODBuffer, 0x69, sizeof(ODBuffer));
      }
      break;
  }
}

uint8_t lwIOLink::SetResponse()
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
      tx_size +=  Pd.In.Size;
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
  txBuffer[checksum_offset] = GetChecksum(txBuffer, tx_size, status.PDIn);
  return tx_size;
}

inline void lwIOLink::ParseMC()
{
  message.channel = (rxBuffer[MCOffset] & 0x60) >> 5;
  message.addr = rxBuffer[MCOffset] & 0x1F;
}

void lwIOLink::begin(Stream &serial,
                    BaudRate baud,
                    unsigned EnablePin,
                    unsigned WakeupPin,
                    int WakeUpMode)
{
  TxEn = EnablePin;
  WuPin = WakeupPin;
  _serial = &serial;
#ifdef ARDUINO_SAM_DUE
  static_cast<UARTClass*>(_serial)->begin(baud, SERIAL_8E1);
#else
  static_cast<HardwareSerial*>(_serial)->begin(baud, SERIAL_8E1); 
#endif
  initTransciever(WakeUpMode);  
}

void lwIOLink::checkIOLinkMasterMsg (uint8_t NewByte)
{
  rxBuffer[rxCnt++] = NewByte;
  if (rxCnt == CKTFrameOffset )
  {
    //Get MSeqType Bits (Figure A.2  IO-Link Spec)
    MSeqType mSeqType = static_cast<MSeqType>(rxBuffer[CKTFrameOffset ] >> 6);
    MasterAccess = static_cast<MCAccess>(rxBuffer[MCOffset] >> 7); //First Uart frame is MC, check IO-Link spec  Figure A.1

    uint8_t od_size = 0;
    uint8_t pd_size = 0;
    if (deviceMode == start)
    {
      if (MasterAccess == MCAccess::Write)
      {
        od_size =  ODSize.Startup;
      }
    }
    else if (deviceMode == preoperate)
    {
      if (MasterAccess == MCAccess::Write)
      {
        od_size = ODSize.Preop;
      }
    }
    //Operate mode
    else
    {
      if (MasterAccess == MCAccess::Write)
      {
        od_size = ODSize.Op;
      }
      pd_size = Pd.Out.Size;
    }
    ExpectedRXCnt = MasterMetadataOffset + od_size + pd_size;
  }
  if (rxCnt == ExpectedRXCnt)
  {
    newIOLinkMasterFrame_recieved = true;
  }
}

inline void lwIOLink::ResetRX()
{
  rxCnt = 0;
  ExpectedRXCnt = 0xFF;
  while (_serial->available() > 0)
  {
    _serial->read();
  }
}

void lwIOLink::initTransciever(int wakeup_mode)
{
  pinMode(TxEn, OUTPUT);
  pinMode(WuPin, INPUT_PULLUP);
  digitalWrite(TxEn, HIGH);
  attachInterrupt(digitalPinToInterrupt(WuPin), WakeupIRQ, wakeup_mode );
}

void lwIOLink::DeviceRsp(uint8_t *data, uint8_t len)
{
  digitalWrite(TxEn, HIGH);
  for (uint8_t i = 0; i < len; i++)
  {
    _serial->write(*(data + i));
  }
  _serial->flush(); //Wait until transmission is finished
  digitalWrite(TxEn, LOW);
  ResetRX();
}

void lwIOLink::run()
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
      if (_serial->available() > 0)
      {
        uint8_t rx_byte = _serial->read();
        if (rx_byte == 0xA2)
        {
          deviceState = run_mode;
          checkIOLinkMasterMsg(rx_byte);
        }
      }
      break;
    case run_mode:
      unsigned long current_time = micros();
      if (_serial->available() > 0)
      {
        uint8_t buff = _serial->read();
        checkIOLinkMasterMsg(buff);
      }
      if( deviceMode == operate
         && (current_time - LastMessage) > CurrentCycleTime )
      {
          deviceMode = start;
          deviceState = wait_wake;
          ResetRX();
          digitalWrite(TxEn, HIGH);
      }
      else if (newIOLinkMasterFrame_recieved)
      {
        newIOLinkMasterFrame_recieved = false;
        ProcessMessage();
        uint8_t tx_size = SetResponse();
        DeviceRsp(txBuffer, tx_size);
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