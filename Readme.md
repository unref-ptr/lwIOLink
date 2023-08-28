# lwIOLink 

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-green.svg)](https://www.gnu.org/licenses/gpl-3.0)
![version](https://img.shields.io/badge/version-0.2.1-blue)

[!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://www.buymeacoffee.com/unrefptr)


*A Light weight IO-Link device communication library for Arduino.*

This library reduces the complexity of the IO-Link communication to send (PDIn) and receive data (PDOut) over an IO-Link Master. The reason the library is called "light weight" is because it simplifies the communication and some of the "complex" features are missing. It must be noted that the aim of this library was not to follow the spec. but to simplify the communication by "dumbing down" the protocol. Hence this library can be used to understand how IO-Link works in a basic way, for commercial development
please look for commercial alternatives as this library will not scale and is not optimized for production environments.

## Features

- Send PDIn
- Recieve PDOut
- Configurable Cycletime
- Recover device after disconnection
- Events

## Caveats

- The scope of this library is to build light-weight IO-Link devices.
- It is not intended to use for production as it will probably not conform to the spec.
- The library is not optimized for real-time applications as it polls for serial IO-Link data.
- The library is not intended to be use to read IO-Link devices. You will need to develop your own IO-Link Master stack (which is out of the scope of this project).
- There is no IODD available. The library is used to test the communication protocol. Please consult the IO-Link [IODD spec](https://io-link.com/share/Downloads/Spec-IODD/IO_Device_Description_V1.1_Specification.zip) to generate your own IODD.


### TODO

#### Low Priority

- ISDU: ISDU processing requires to follow the IO-Link spec. and its state machines...left to any highly motivated developer out there to contribute ;)
- Data Storage: Depends on ISDU, and it requires a HAL to access non-volatile storage.

## Tested Arduino Targets 

* UNO     (COM1,COM2)
* ESP32   (COM1,COM2,COM3)
* RPI Pico (COM1,COM2)

If you find that the library works for another Arduino board or does not work, notify me by opening an Issue.

Note: Some MCUs might not work in COM3 due to the timing constraings of the IO-Link spec. and the non-optimized Arduino Core libraries.

## How to Use

You can test the library by using the demo located in the [examples dir](https://github.com/unref-ptr/lwIOLink/tree/main/examples). In general the user has to define the process data size, and the cycle time. Whenever the device is in operate mode data from the master can be retrieved (PDOut) by implementing the callback function `lwIOLink::OnNewCycle()`. 

Its important that in the arduino `loop()` no blocking functions or functions that take a long time to execute (e.g. communication over I2C, reading ADC,etc) are not executed. This is because this library works in a polling fashion and is always looking for serial data to communicate with the IO-Link Master. The reason for this is that the
Arduino Core API does not have a way to access Serial data over interrupts, in addition the API is not really standardized over all the cores so I tried to make it as simple as possible for portability reasons (within the Arduino ecosystem). Instead execute all your logic that could block this library in `lwIOLink::OnNewCycle()`.

The [cycle time](https://github.com/unref-ptr/lwIOLink/blob/ea4fb8b749b550778cd0273a291457a5b9b49628/src/lwIOLink.hpp#L113) should be set to the minimum required to get data for your arduino (for example reading an ADC, setting an RGB led stripe,etc). This is due to the fact that IO-Link can lose communication if the device does not reply quick enough. It is important to mention that `lwIOLink::Device::SetPDIn()` can be updated ( at any time, but will only be set if the device is in operate mode. For a more details about the api check the header of the library.

## Hardware

Hardware-wise IO-Link requires an IO-Link device transceiver. Tested with an LT3669 transciever but theoretically any transciever, that has a UART interface (TX,RX), a Wakeup pin (for notification via interrupt) and an TXEN pin (pin that has to be pulled high to send data), should work. 

## Contributions

You are welcomed to make contributions by creating a pull request or sending your patch with information about the changes to unref-ptr@protonmail.com .

