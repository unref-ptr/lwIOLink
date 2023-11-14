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
#include <lwIOLink.hpp>

using namespace lwIOLink;

static  unsigned constexpr PDInSize = 2;
static  unsigned constexpr PDOutSize = 2;
static unsigned constexpr min_cycle_time = 50000; //Cycle time in microseconds for operate mode
uint8_t PDOut[PDOutSize] = {0, 0}; //Buffer that recieves data from the Master
uint8_t PDIn[PDInSize] = {0, 1}; //Buffer which will be sent to the Master

//Hardware configuration for the device
constexpr Device::HWConfig HWCfg =
{
#if defined(ARDUINO_ARCH_AVR)
  .SerialPort = Serial,
#else
  .SerialPort = Serial2,
#endif
  .Baud = lwIOLink::COM2,
  .WakeupMode = FALLING,
  .Pin =
  {
    .TxEN = 18,
    .Wakeup = 5,
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO)
    .Tx = 17,
    .Rx = 16
#endif
  }
};

Device iol_device( PDInSize, PDOutSize, min_cycle_time);

/*
 * Optional static callback to know when Master has read events
 */
void Device::OnEventsProcessed()
{
  Serial.println("Events Processed by Master!");
}

void Device::OnNewCycle()
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

void setup()
{
  Serial.begin(115200);
  iol_device.begin(HWCfg);
  /* Example API of how to Set IO-Link Events
  */
  uint16_t EventCode = 0x1234;
  // Note: Cannot Set events in ISR, not concurrent safe
  Event::POD MyEvent(Event::Qualifier::Application, Event::Qualifier::SingleShot, EventCode);
  Device::EventResult result = iol_device.SetEvent(MyEvent);
  (void)result; //Normally you would check that the Event could be set
}

void loop()
{
  iol_device.run();
}
