/*
TwoSerialWrap.h
Simply wraps a hardware and Software Serial (to be set up separately)
to have one IO interface.

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

#ifndef TwoSerialWrap_h
#define TwoSerialWrap_h

#include <inttypes.h>
#include <Stream.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
/******************************************************************************
* Definitions
******************************************************************************/
class TwoSerialWrap : public Stream
{
private:
  SoftwareSerial *SWUart;
  HardwareSerial *HWUart;
  unsigned long HWBaud;
  unsigned long SWBaud;
public:
  // public methods
  TwoSerialWrap::TwoSerialWrap(HardwareSerial *pHWUart, unsigned long HWBaud, SoftwareSerial *pSWUart, unsigned long SWBaud);
  ~TwoSerialWrap();
  bool running;
  void end();
  void go();
  int peek();
  size_t write(uint8_t byte);
  int read();
  int available();
  void flush();
  operator bool() { return true; }
  
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};
extern TwoSerialWrap TSerial;

#endif
