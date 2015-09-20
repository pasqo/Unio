# UNIO
*UNIO Protocol Library for Arduino, AVR 8 bit Microntrollers.*

## Overview
UNI/O™ is a one wire bidirectional serial protocol used in Microchip EEPROMs operating at clock frequencies between 10 KHz and 100 KHz.

This library is an implementation of the protocol for Arduino boards based on 8 bit AVR microntrollers as well as standalone AVR microntrollers.

The library features a small size footprint and a robust implementation that decouples code and serial hardware communication timing by synchronizing IO operations to an internally synthesized clocked derived from one of the available 8 bit timers in the target AVR. This enables operation at the full operating frequency range, up to 100KHz.

The API includes all the main memory operations outlined in the [Microchip 11XX010 EEPROM family datasheet](http://www.microchip.com/wwwproducts/Devices.aspx?product=11LC010), as well as other common operations such as memory dump, load, etc.

## Example
The library includes an interactive serial console application called UnioShell that makes use of all the API through a simple set of documented commands.

To use UnioShell with an EEPROM, simply change the code to reflect the Arduino digital pin connected to the SCIO serial line, compile, upload, and open the Arduino serial console (or your preferred serial console application, I use CoolTerm) configured as 115200 8N1. It is best to use line mode if available with LF only termination.

For convenience the UnioShell example code is reported here.
```cpp
// Use macro SCIO_PIN to specify the arduino digital pin connected to SCIO.
// Here the EEPROM SCIO pin is connected to digital pin D2.
#define SCIO_PIN 2
#include "AvrMap.h"
#include "UnioShell.h"

void setup ()
{
  Serial.begin (115200);
  UnioShell shell;
}

void loop () {}
```

## AVR Timer Resource Sharing
The library uses Timer/Counter2 in the ATmega328/ATmega2560 based boards, Timer/Counter3 in the ATmega32U4 based boards, and Timer/Counter1 in standalone ATtiny boards.
These timers maybe used by Wiring to implement functionality so that functionality will not be available and may conflict with the operation of the library.
For instance Timer/Counter2 is usedin Wiring to implement the tone() function in boards based on the ATmega328p.

## Dependencies
This library depends on the https://github.com/pasqo/AvrMap library to make the code portable across supported arduino boards and AVR microcontrollers, please install it alongside to the Unio library.

## Portability
Although the code should work for all supported boards I have only been able to test it on boards with ATmega328p AVRs.
Reports of successful usage on other boards, or any issues, are welcome.

## Links
- [Microchip UNI/O™ Serial EEPROMS](http://www.microchip.com/pagehandler/en-us/products/memory/serialeeprom/unio.html)
- [Microchip 11LC010 datasheet](http://www.microchip.com/wwwproducts/Devices.aspx?product=11LC010)