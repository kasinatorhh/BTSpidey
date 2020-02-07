/*
TwoSerialWrap.cpp (formerly NewSoftSerial.cpp) - 

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
// 
// Includes
// 
#include <Arduino.h>
#include "TwoSerialWrap.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
//
// Statics
//
//
// Private methods
//
//
// Constructor
//
TwoSerialWrap::TwoSerialWrap(HardwareSerial *pHWUart, unsigned long HWBaud, SoftwareSerial *pSWUart, unsigned long SWBaud)
{
  HWUart=pHWUart;
  SWUart=pSWUart;
}

//
// Destructor
//
TwoSerialWrap::~TwoSerialWrap()
{
}

//
// Public methods
//

// Read data from buffer
int TwoSerialWrap::read()
{
  if (SWUart->available()>0){
    return SWUart->read();
  }
  if (HWUart->available()>0){
    return HWUart->read();
  }
}

int TwoSerialWrap::available()
{
  if (HWUart->available()>0) return HWUart->available();
  if (SWUart->available()>0) return SWUart->available();
  return 0;
}

size_t TwoSerialWrap::write(uint8_t b)
{
  HWUart->write(b);
  return SWUart->write(b);
}

void TwoSerialWrap::end(){
  HWUart->end();
  SWUart->end();
  running=false;
}

void TwoSerialWrap::go(){
  HWUart->begin(HWBaud);
  SWUart->begin(SWBaud);
  running=true;
}

void TwoSerialWrap::flush()
{
  // There is no tx buffering, simply return
}

int TwoSerialWrap::peek()
{
  if (HWUart->available())return HWUart->peek();
  if (SWUart->available())return SWUart->peek();
  return -1;
}
