//////////////////////////////////////////////////////////////////////////////
// UNIO Protocol Library for Arduino, AVR 8 bit Microntrollers.
//
// Copyright (C) 2014 Pasquale Cocchini <pasquale.cocchini@ieee.org>
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//////////////////////////////////////////////////////////////////////////////

#ifndef UnioLib_h
#define UnioLib_h

#define PASQO_UNIO_MAJOR 1
#define PASQO_UNIO_MINOR 0

#include <Arduino.h>
#include "AvrMap.h"

//////////////////////////////////////////////////////////////////////////////
// PC: This library makes use of one AVR 8 bit timer/counter for bitbang 
// synchronization. We use, in order of availability in the AVR architecture,
// TCNT2 (ATmega328, ATmega2560), TCNT3 (ATmega32U4), or TCNT1 (ATtiny). 
// Although TCNT3 is a 16 bit counter, here we use it as an 8 bit one.
// Functions relying on the used timer by Wiring will be affected. For
// instance TCNT2 in the ATmega328p is used by the Arduino tone() function,
// so that function will not work.
//////////////////////////////////////////////////////////////////////////////
#if defined TIFR2
  #define TCCRnA TCCR2A
  #define TCCRnB TCCR2B
  #define  OCRnA OCR2A
  #define  OCFnA OCF2A
  #define  TIFRn TIFR2
  #define  WGMn1 WGM21
  #define   CSn1 CS21
#elif defined TIFR3
  #define TCCRnA TCCR3A
  #define TCCRnB TCCR3B
  #define  OCRnA OCR3A
  #define  OCFnA OCF3A
  #define  TIFRn TIFR3
  #define  WGMn1 WGM31
  #define   CSn1 CS31
#elif defined TIFR1
  #define TCCRnA TCCR1A
  #define TCCRnB TCCR1B
  #define  OCRnA OCR1A
  #define  OCFnA OCF1A
  #define  TIFRn TIFR1
  #define  WGMn1 WGM11
  #define   CSn1 CS11
#else
#error Unsupported MCU architecture.
#endif

//////////////////////////////////////////////////////////////////////////////
// PC: The SCIO pin must be assigned through the SCIO_PIN define before the
// library header inclusion in user code (please see examples). The pin is
// then statically resolved here at compile time to the corresponding AVR IO
// registers for the most efficient operation.
//////////////////////////////////////////////////////////////////////////////
#ifndef SCIO_PIN
#error Missing SCIO_PIN <digital-pin> define before library include.
#else // SCIO_PIN
#ifndef PORTMap
#error Please install library "https://github.com/pasqo/AvrMap"
#endif
#define DDRX   DDRMap(SCIO_PIN)
#define PINX   PINMap(SCIO_PIN)
#define PORTX  PORTMap(SCIO_PIN)
#define SCIO   1<<BITMap(SCIO_PIN)
#endif // SCIO_PIN

//////////////////////////////////////////////////////////////////////////////
// PC: Uncomment the following for approximately 500 bytes of smaller code
// but with maximum speed of 50KHz (20us bit period TE), instead of 100KHz.
//#define UNIO_NO_HIGH_SPEED
//////////////////////////////////////////////////////////////////////////////
#ifndef UNIO_NO_HIGH_SPEED
#define AI __attribute__((always_inline))
#else
#define AI
#endif // UNIO_NO_HIGH_SPEED

//////////////////////////////////////////////////////////////////////////////
// The description of the UNIO protocol and functional and timing parameters
// are taken from the 1K-16K UNI/O Serial EEPROM Family Data Sheet. Please
// refer to that document for a more detailed explanation of the operation
// of the device. Here I encapsulate all code within the UNIO class. Note
// how the struct keyword is used rather than class to have public as default.
//////////////////////////////////////////////////////////////////////////////
struct Unio
{
  enum OpCode
  {
    // Read data from memory array beginning at specified address.
    READ  = 0x03U,
    // Read data from current location in memory array.
    CRRD  = 0x06U,
    // Write data to memory array beginning at specified address.
    WRITE = 0x6CU,
    // Set the write enable latch (enable write operations).
    WREN  = 0x96U,
    // Reset the write enable latch (disable write operations).
    WRDI  = 0x91U,
    // Read STATUS register.
    RDSR  = 0x05U,
    // Write STATUS register.
    WRSR  = 0x6EU,
    // Write 0x00 to entire array.
    ERAL  = 0x6DU,
    // Write 0xFF to entire array.
    SETAL = 0x67U,
    // Device address for 11xxxx0 family.
    ADDR  = 0xa0U
  };

  enum Timing
  {
    TSTBY  = 600, // Min standby pulse time [us].
    TSS    =  10, // Min start header set-up time [us].
    THDR   =   5, // Min start header low pulse time [us].
    TE_MIN =  10, // Min bit period [us].
    TE_MAX = 100, // Max bit period [us].
    TE     =  20, // Default period [us].
    TIJIT  = 200, // Input edge jitter tolerance +-0.06 TE.
                  // When writing we need to make sure our edge is
                  // within 200ns of ideal half TE.
    TOJT   =   3, // Output edge jitter +-0.25 TE.
                  // When sampling the slave we need to sample at
                  // TE - 1/4TE to avoid TOJT.
    TSP    =  50, // Max input filter spike suppression [ns].
    WC     =   5, // Max write cycle time [ms] for WRITE and WRSR.
    WCA    =  10, // Max write cycle time [ms] for ERAL and SETAL.
  };

  enum Handshake
  {
    HEADER = 0x55, // Header synchronization byte.
    MAK    =    1, // A LH edge at half period.
    NOMAK  =    0, // A HL edge at half period.
    SAK    =    1, // A LH edge at half period.
    NOSAK  =    0, // Absence of an LH edge at half period.
  };

  enum StatusRegister
  {
    RDSR_WIP = 0x00U,
    RDSR_WEL = 0x01U,
    RDSR_BP0 = 0x02U,
    RDSR_BP1 = 0x04U,
  };

  enum State
  {
    IDLE    = 0,
    STANDBY = 1
  };

  ////////////////////////////////////////////////////////////////////////////
  // UNIO public interface.
  ////////////////////////////////////////////////////////////////////////////
  // PC: All commands attempt at leaving the device in standby state with the
  // bus driven high. This enables the concatenation of multiple commands
  // without requiring the use of a standby pulse. A sequence of commands 
  // shall always be initiated with a call to Standby after an error that 
  // causes the device to revert to idle mode.
  ////////////////////////////////////////////////////////////////////////////

  // UNIO Constructor.
  // device_addr: the UNIO device address, default is for 11xxxx0 family.
  // period_us: the bus period in us, default is 20us.
  Unio (byte device_addr = ADDR, byte period_us = TE) 
    : m_addr (device_addr), m_tick (1), m_stby (IDLE), m_period_us (0)
  {
    // PC: At power-on the device is in shutdown state and requires an LH
    // edge on SCIO to enter idle mode which we do in WakeUp.
    WakeUp ();
    Init (period_us);
  }

  // Initialize driver.
  // Explicitly initialize the driver with specified bus period.
  void Init (byte period_us)
  {
    BusPeriod (period_us);
    StartBusClock ();
    StandBy ();
  }

  // PC: When calling this method we maybe in idle mode or standby with
  // the bus driven high or read mode if after an error. To enter standby
  // mode we generate a TSTBY LH pulse and stay driven high.
  void StandBy () 
  {
    WrScio (0);
    SetToWr ();
    delayMicroseconds (10);
    WrScio (1);
    delayMicroseconds (TSTBY);
    m_stby = STANDBY;
  }

  // StandBy state Check.
  // Returns true if the bus is in standby state.
  bool IsStandBy () { return m_stby == STANDBY; }

  // Idle state Check.
  // Returns true if the bus is in idle state.
  bool IsIdle () { return m_stby == IDLE; }

  // Read data from device.
  // Read len bytes from the device starting at address addr and store in the
  // buffer memory pointed by buf.
  bool Read (byte *buf, word addr, word len)
  {
    byte cmd[] = { HEADER, m_addr, READ, addr >> 8, addr & 0xff };
    return Transfer (cmd, 4, buf, len, NULL, 0);
  }

  // Write data to device.
  // Write len bytes in the device starting at address addr taken from the
  // buffer memory pointed by buf. Low level writes cannot go across a frame
  // so write is segmented accordingly.
  bool Write (const byte *buf, word addr, word len)
  {
    if (IsStandBy())
    {
      word wlen;
      while (len)
      {
	wlen = len;
	if ((addr & 0x0f) + wlen > 16)
	  wlen = 16 - (addr & 0x0f);
	EnableWrite ();
	PageWrite (buf, addr, wlen);
	if (!WaitForWrite ()) 
	  break;
	buf += wlen;
	addr += wlen;
	len -= wlen;
      }
    }
    return m_stby;
  }

  // Read status bit register.
  // Read the status bit register in the byte pointed by status.
  bool ReadStatus (byte *status)
  {
    byte cmd[] = { HEADER, m_addr, RDSR };
    return Transfer (cmd, 2, status, 1, NULL, 0);
  }

  // Write status bit register.
  // Write the status bit register with the byte status.
  bool WriteStatus (byte status)
  {
    byte cmd[] = { HEADER, m_addr, WRSR, status };
    return Transfer (cmd, 3, NULL, 0, NULL, 0);
  }

  // Clear device memory.
  // Overwrite the entire device memory with zeros.
  // The call blocks until the write operation has completed.
  bool EraseAll ()
  {
    EnableWrite();
    byte cmd[] = { HEADER, m_addr, ERAL };
    Transfer (cmd, 2, NULL, 0, NULL, 0);
    WaitForWrite ();
    return m_stby;
  }

  // Set device memory.
  // Overwrite the entire device memory with ones.
  // The call blocks until the write operation has completed.
  bool SetAll ()
  {
    EnableWrite();
    byte cmd[] = { HEADER, m_addr, SETAL };
    Transfer (cmd, 2, NULL, 0, NULL, 0);
    WaitForWrite ();
    return m_stby;
  }

  protected:

  ////////////////////////////////////////////////////////////////////////////
  // Driver implementation, accessible by derived classes.
  ////////////////////////////////////////////////////////////////////////////

  // Wake up from power-on with a LH transition leaving the bus high.
  void WakeUp ()
  {
    WrScio (0);
    SetToWr ();
    SyncToStart ();
    WrScio (1);
  }

  // PC: We choose to decouple the bus timing from the code and use a timer
  // to syncronize bit operations to avoid drifting in long operations and
  // general timing sensitivity with code. The assumption here is that while
  // engaged in bus operations that are synchronized to a timer generated
  // clock TC, the code executed in between TC clock edges must take less 
  // than the time taken to generate the next clock edge, otherwise we miss
  // it and we go out of synch. 
  // Because of the high bus slave jitter equal to plus minus one quarter of
  // the device bit period TE our timer half period THP needs to be at least
  // one quarter of TE to sample slave answers safely. We use counter TCNT2 
  // with a prescale of 8 yielding a counter period of 0.5us for an MCU 16MHz
  // clock and a THP that can be therefore programmed between 0.5us and 128us
  // with a 8bit register. The THP needed for the device allowed TE range of
  // [10us, 100us] is then [2.5us, 25us] which is well inside the counter 
  // range. But any value of TE in this range must be a multiple of 2 because
  // of the timer discrete 0.5us quantum, so TE can be in [10, 12, ..., 100].
  // As for how much code we can write in between edges, in the fastest 10us
  // case we have approximately 40 MCU clock cycles (8 MCU cycles for five
  // prescaled counter ticks) for code execution before we go out of sync.

  // Here we setup the timer for the bus clock which will be resetted
  // after the count reaches what is stored in the compare register OCR2A.
  void BusPeriod (byte period_us)
  {
    // Round off to multiple of 2.
    period_us &= 0xFEU;
    if (period_us < TE_MIN)
      period_us = TE_MIN;
    else if (period_us > TE_MAX)
      period_us = TE_MAX;
    // Set the counter in CTC (clear timer on compare match) mode.
    TCCRnA = 1<<WGMn1;
    // Set the compare register to the number yielding requested THP ticks.
    OCRnA = period_us / 2 - 1;
    // At counter match TIFRn & 1<<OCFnA will be set and we need to clear it
    // explicitly since we have not installed an interrupt routine.
    m_period_us = period_us;
  }

  byte BusPeriod () const { return m_period_us; }

  // Start TCNTn with a prescaler of 8. Since we are running at 16MHz
  // this provides a counter tick of 0.5ns we use to synchronize at every
  // quarter of bus clock period.
  void StartBusClock () { TCCRnB = 1<<CSn1; }

  void StopBusClock () { TCCRnB &= ~(1<<CSn1); }

  bool EnableWrite ()
  {
    byte cmd[] = { HEADER, m_addr, WREN };
    return Transfer (cmd, 2, NULL, 0, NULL, 0);
  }

  bool DisableWrite ()
  {
    byte cmd[] = { HEADER, m_addr, WRDI };
    return Transfer (cmd, 2, NULL, 0, NULL, 0);
  }

  // PC: It is always called within a page boundary.
  bool PageWrite (const byte *buf, word addr, word len)
  {
    byte cmd[] = { HEADER, m_addr, WRITE, addr >> 8, addr & 0xff };
    return Transfer (cmd, 4, NULL, 0, buf, len);
  }

  // Block until pending writes are completed.
  bool WaitForWrite ()
  {
    byte status;
    bool rs = true;
    do { rs = ReadStatus (&status); }
    while (rs && (status & 1<<RDSR_WIP));
    return rs;
  }

  private:

  ////////////////////////////////////////////////////////////////////////////
  // Driver implementation, not accessible by derived classes.
  ////////////////////////////////////////////////////////////////////////////

  // In general, compiler optimizations and real time applications don't mix
  // well and one needs to be careful checking the produced assembly code or
  // using a scope (as in my case) to make sure any reordering has not 
  // occurred in the wrong place. One could change compiler flags (not 
  // possible in Arduino IDE currently for 1.0.5) or use compile attributes
  // or pragmas in the code (not available in this Arduino IDE avr-gcc 
  // version). Using volatile may help but is not panacea. Using general 
  // purpose io registers such as the GPIORx on the AVR also helps. Right now
  // keeping the code linear and forcing inlining on nested calls let us 
  // achieve the maximum 100KHz speed.

  // We need to model 4 ticks in a bus clock cycle to properly write and
  // sample the slave answers. At start of a cycle we are at tick 1 and at 
  // end at tick 4 (actually reversed in counter).
  // This method advances timing by the specified ticks from the current one.
  void AI SyncAtNext (byte ticks)
  {
    m_tick -= ticks;
    while (ticks--)
      SyncToEdge ();
  }

  // Here at any point we can easily synchronize to the start of the
  // next bit period.
  void AI SyncToStart ()
  {
    while (m_tick--)
      SyncToEdge ();
    m_tick = 4;
  }

  void ResetBusClock () { m_tick = 1; }

  // Wait for the counter to reach the predefined top and then clear the 
  // flag by writing a logic one.
  void AI SyncToEdge ()
  {
    while (!(TIFRn & 1<<OCFnA));
    TIFRn |= 1<<OCFnA;
  }

  void SetToWr () { DDRX |= SCIO; }

  void SetToRd () { DDRX &= ~SCIO; }

  void WrScio (bool b) { b ? PORTX |= SCIO : PORTX &= ~SCIO; }

  void TgScio () { PORTX ^= SCIO; }

  bool RdScio () { return PINX & SCIO; }

  void Header ()
  {
    //delayMicroseconds (TSS);
    //WrScio (0);
    //delayMicroseconds (THDR);

    // PC: TSS (10us) is actually the same as the minimum bit period TE so
    // we can accomplish set-up setting the bus high for one clock cycle.
    SyncToStart();
    WrScio (1);
    // PC: THDR (5us) is actually half of the minimum bit period TE (10us)
    // so we accomplish the THDR low pulse by simply writing a synchronized
    // low bit.
    WrBit (0);
  }

  // Bit writes and reads are synchronized to the synthesized bus clock.
  // For a write we first sync to the next bit period start, than we write
  // the negated bit value, set the port direction to write, advance to the
  // middle of the bit period and then write the bit value to encode the
  // bit value with manchester encoding. The bit write might be preceded by
  // both a write or a Read. In the first case setting the port direction
  // to write again has no effect but to use an extra MCU clock cycle. In
  // the second case though the value presently stored in the PORTXY maybe
  // either a 1 if the master previously wrote a MAK or 0 if NoMAK. If the
  // the value was a NoMAK than writing on PORTXY a zero while in Read mode
  // has no effect. If there was a 1 writing on PORTXY will cause the pull-up
  // resistor on SCIO to turn on for one clock cycle before being turned off
  // immediately at the port direction switch to out. But In this case this
  // does not present a problem electrically, since the pull-up starts driving
  // the SCIO bus high and then the strong write high follows. Having the
  // port direction synchronized inside this function simplifies the code.
  void AI WrBit (bool b)
  {
    SyncToStart ();
    WrScio (!b);
    SetToWr();
    SyncAtNext (2);
    TgScio ();
  }

  // Here too we first synchronize to the start of the bit period first.
  // Then we change the port direction to Read from what it was before,
  // either Read or write. If it was Read, there is no effect but just
  // using one MCU clock cycle. If it was write with a zero in the PORTXY
  // register that this would not have any effect either. If there was a
  // 1 we would be setting a pull-up on the line. This would not disturb
  // the line since the electrical values are the same but if left unchanged
  // it would require the device to spend more energy to drive the bus low 
  // also taking longer time. To avoid this we write 0 in the PORTXY register
  // right away to turn off the pull-up. If the previous value was 0 already
  // then writing 0 has no effect. To read the SAK we prefer the pull-up on
  // since the device seems to leave the line undriven for NOSAK.
  // The logic for returning NoSAK works since false will also be returned 
  // for when we have matching logic levels at 1/4 and 3/4 T.
  bool AI RdBit (byte sak = NOSAK)
  {
    bool a, b;
    SyncToStart ();
    SetToRd();
    //WrScio (sak); // This seems redundant after all.
    SyncAtNext (1);
    a = RdScio ();
    SyncAtNext (2);
    b = RdScio ();
    return !a && b;
  }

  // When ending a command with NoMAK and after a successful sak the device
  // is still in standby mode and ready to receive another command without
  // requiring a new standby pulse. If this is the case we complete the 
  // previous sak bit period and then drive the bus high right away. Then
  // for a consecutive command this call will be followed by another command
  // initiated by a start header only.
  bool AI MakSak (bool mak)
  {
    WrBit (mak);
    bool rs = RdBit (SAK);
    if (!mak)
    {
      SyncToStart ();
      WrScio (1);
      SetToWr ();
    }
    return rs;
  }

  bool AI WrByte (byte b, bool mak)
  {
    byte i = 8;
    while (i--)
    {
      WrBit (b & 0x80);
      b <<= 1;
    }
    return MakSak (mak);
  }

  bool AI RdByte (byte *b, bool mak)
  {
    byte data = 0, i = 8;
    while (i--)
      data = (data << 1) | RdBit ();
    *b = data;
    return MakSak (mak);
  }

  // PC: This flattens the code as much as possible. With no forced inlines
  // at 20 us we notice a slight borrowing when switching wr and rd bytes.
  // At 10 us the borrowing is larger than a quarter period and interferes
  // with the slaves causing failure so we need to inline.
  bool Transfer (
      const byte *cp, word cl, 
            byte *rp, word rl, 
      const byte *wp, word wl)
  {
    if (IsStandBy())
    {
      bool rs = true, mak = rl || wl;
      cli();
      Header();
      WrByte (*cp++, MAK); // The header always gets a NoSAK.
      while (cl-- && rs)
	rs = WrByte (*cp++, cl || mak); // NoMAK only if last byte.
      while (rl-- && rs)
	rs = RdByte (rp++, rl);
      while (wl-- && rs)
	rs = WrByte (*wp++, wl);
      sei();
      m_stby = rs;
    }
    return m_stby;
  }

  private:

  byte m_addr; // Device address.
  byte m_tick; // Bus clock tick counter.
  byte m_stby;
  byte m_period_us;
};
#endif // UnioLib_h
