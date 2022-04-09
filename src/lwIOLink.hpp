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
#define pragma once
#include <stdint.h>
#include <Arduino.h>

class lwIOLink
{
  public:
    /*
       lwIOLink Constructor
       PDIn = total # bytes the device sends (Device->Master)
       PDOut = total # bytes the device gets (Master->Device)
       min_cycletime = minimum cycle time in us  (See B.1.3)
    */
    lwIOLink(uint8_t PDIn, uint8_t PDOut,uint32_t min_cycletime);
    //Modes for the Device
    enum Mode
    {
        start,
        preoperate,
        operate
    };
    enum BaudRate: uint32_t
    {
        COM1 = 4800,
        COM2 = 38400,
        COM3 = 230400
    };
    //Table A.5
    enum PDStatus
    {
        Valid = 0,
        Invalid = 1
    };
    struct HWConfig
    {
        Stream &SerialPort; 		 //reference to Arduino Serial port
        BaudRate Baud;			// baud rate for transmission
        int WakeupMode; 	/* Type of interrupt to detect wakeup from transceiver   
                        FALLING, RISING */
        struct Pin_t
        {
            unsigned TxEN;  	 //Digital Output pin used to enable TX of data
            unsigned Wakeup;	/* Digital Input pin used to get Wakeup Requests
                        Must be interrupt capable. */
#ifdef ARDUINO_ARCH_ESP32
            /* Most ESP32 Dev kits dont have default UART
              pins exposed. Hence need to configure available pins                   
            */
            unsigned Tx;	
            unsigned Rx;
#endif //ARDUINO_ARCH_ESP32
        };
        Pin_t Pin;
    };
    /*
       Begin the IOLink Device Hardware
    */
    void begin(const HWConfig config);
    /*
       Run the IOLink Device, should be called in a loop
    */
    void run();
    /*
      Get the Process Data Output
      buffer = Pointer to Buffer where to store PDOut
      pStats = Pointer to memory where status of PDOut will be saved
      Returns false if the device is not in operate
    */
    bool GetPDOut(uint8_t * buffer, PDStatus * pStatus) const;
    /*
      Sets the Process Data Input
      Returns false if the device is not in operate
      or the len > than the available PDIn Size
    */
    bool SetPDIn(uint8_t * pData, uint8_t len);
    /* 
     *  Sets the validity of PDIn
     *  Return false if not in operate mode
     *  By default the PDIN status is set to valid
     *  See Table A.5
     */
    bool SetPDInStatus(PDStatus pd_status);
    // Gets the current Mode of the IOLink device
    Mode GetMode() const;
    /* Callback for whenever an IOLink Cycle is completed in operate mode */
    static void OnNewCycle(void);
  private:
    static unsigned constexpr MaxPD = 32;
    static unsigned constexpr MaxOD = 32;
    static unsigned constexpr SizeDP1 = 16;
    static unsigned constexpr ChecksumSize = 1;
    static unsigned constexpr MCSize = 1;
    static unsigned constexpr MaxIOLMsgSize =  MCSize + ChecksumSize + MaxOD + MaxPD; 
    static unsigned constexpr MasterMetadataOffset = MCSize + ChecksumSize;
    static unsigned constexpr MCOffset = 0;
    static unsigned constexpr CKTFrameOffset = 1;
    /* Cycle time constants See B.3*/
    static unsigned constexpr TotalTimeEncodings = 3;
    static uint32_t constexpr TimeBaseLUT[TotalTimeEncodings] = {100, 400, 1600}; //us
    static uint32_t constexpr TimeOffsetLUT[TotalTimeEncodings] = {0,6400,32000}; //us

    //ISDU Not implemented
    static bool constexpr ISDUSupported = false;
    //Message struct for IO-Link
    struct ioLink_message_t
    {
        uint8_t channel;
        uint8_t addr;
    };

    enum Channel
    {
        PROCESS = 0,
        PAGE ,
        DIAGNOSIS,
        ISDU
    };
    struct ODSize_t
    {
        uint8_t Startup;
        uint8_t Preop;
        uint8_t Op;
    };
    //OD Size for the device (1,2,8 or 32)
    static ODSize_t constexpr ODSize =
    {
        .Startup = 1,
        .Preop = 8,
        .Op = 2
    };
    //Table A.2
    enum class MCAccess
    {
        Read = 1,
        Write = 0
    };

    //Table A.3
    enum class MSeqType
    {
        Type0 = 0,
        Type1,
        Type2
    };
    //States for interpreting IO-Link data
    enum ioLink_states
    {
        wait_wake, //Waiting for wakeup signal
        wait_valid_frame, //Waiting for a valid startup message
        run_mode  //Master started communication with device
    };

    //Table B.2
    enum MasterCommands
    {
        MasterIdent = 0x95,
        DeviceIden = 0x96,
        DeviceStartup = 0x97,
        PDOutOperate = 0x98,
        Operate = 0x99,
        Preoperate = 0x9A,
    };
    /* Table B.1 */
    enum DP1_Param
    {
        MasterCommand = 0x00,
        MasterCycleTime,
        MinCycleTime,
        MSeqCap,
        RevisionID,
        ProcessDataIn,
        ProcessDataOut,
        VID1,
        VID2,
        VID3,
        DID1,
        DID2,
        DID3,
        FID1,
        FID2,
        Res,
        SystemCommand
    };

    struct Status
    {
        PDStatus PDIn;
        PDStatus PDOut;
    };

    struct PDBuffer
    {
        uint8_t Data[MaxPD];
        size_t Size;
    };

    struct PD
    {
        PDBuffer Out;
        PDBuffer In;
    };
    uint8_t GetMasterTXSize();
    //Reset the RX Line
    void ResetRX();
    //Get the MSeq Cap according B.1.4
    uint8_t GetMseqCap() const;
    //Process an incoming Master Message
    void ProcessMessage();
    //Sets the Response for the Master, returns the total bytes to send
    uint8_t SetResponse();
    uint8_t rxBuffer[MaxIOLMsgSize];
    uint8_t txBuffer[MaxIOLMsgSize];
    void initTransciever(int wakeup_mode) const;
    //Parse the MC (Figure A.1)
    void ParseMC();
    //Send device response to master
    void DeviceRsp(uint8_t *data, uint8_t len);
    //Prepare the device message
    void prepareMessage(uint8_t od_size);
    /*
    * Called whenever a new RX frame 
    * from the master is recieved
    * rx_byte = byte recieved from the UART
    */
    void SaveMasterFrame(uint8_t rx_byte);
    //Generate IOLink checksum
    uint8_t GetChecksum(uint8_t *data, uint8_t length, PDStatus status) const;
    //Decode cycletime according to Table B.3, return value time in microseconds
    uint32_t DecodeCycleTime(uint8_t encoded_time) const;
    /* Encode cycletime according to Table B.3
    *  If the cycle time time cannot be encoded 
    *  the nearest greater cycle time will be used.
    *  e.g., cycleTime_us = 1333 us -> encoded value = 1340 us
    */
    uint8_t EncodeCycleTime(uint32_t cycleTime_us) const;
    Stream * SerialPort = nullptr; 
    ioLink_message_t message;
    PD Pd;
    uint8_t ODBuffer[MaxOD];
    /* Process Data status */
    Status status = { .PDIn = Valid, .PDOut = Invalid};
    /* Table B.1 */
    uint8_t ParameterPage1[SizeDP1] = {0};
    bool NewCmd = false;
    unsigned TxEn;
    unsigned WuPin;
    MasterCommands Cmd;
    Mode deviceMode = start;
    MCAccess MasterAccess;
    uint8_t ExpectedRXCnt = 0;
    ioLink_states deviceState = wait_wake;
    bool MasterMsgComplete = false;
    uint8_t rxCnt = 0;
    unsigned long CurrentCycleTime = 0;
    unsigned long LastMessage = 0;
};
