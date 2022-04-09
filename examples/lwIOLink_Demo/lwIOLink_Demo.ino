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
#include <stdint.h>
#include "lwIOLink.hpp"
static  unsigned constexpr PDInSize = 2;
static  unsigned constexpr PDOutSize = 2;
static unsigned constexpr min_cycle_time = 50000; //Cycle time in microseconds for operate mode

uint8_t PDOut[PDOutSize] = {0,0}; //Buffer that recieves data from the Master
uint8_t PDIn[PDInSize] = {0,1}; //Buffer which will be sent to the Master


constexpr lwIOLink::Config_t IOLinkConfig =
{
  .serial = Serial2, 
  .baud = lwIOLink::COM3,
  .WakeupMode = FALLING, 
  .Pin = 
  {
    .TxEN = 18,
    .Wakeup = 5,
#ifdef ARDUINO_ARCH_ESP32
    .Tx = 17,
    .Rx = 16
#endif
  }
};


lwIOLink iol_device( PDInSize, PDOutSize, min_cycle_time);

void lwIOLink::OnNewCycle()
{
  PDStatus PDOutStatus;
  bool pdOut_ok = iol_device.GetPDOut(PDOut, &PDOutStatus);
  if (pdOut_ok && PDOutStatus == Valid)
  {
    //dosomething with pdOut
  }
  //Modify PDIn 
  PDIn[0]++;
  PDIn[1]++;
  bool result = iol_device.SetPDIn(PDIn, sizeof(PDIn));
  if ( result == true)
  {
      iol_device.SetPDInStatus(lwIOLink::Valid);
  }

}

void setup() {
  iol_device.begin(IOLinkConfig);
}

void loop() {
  iol_device.run();

}
