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

#ifndef UnioShell_h
#define UnioShell_h

#include "Unio.h"

struct UnioShell : public Unio
{
  UnioShell (byte device_addr = ADDR, byte period_us = TE) 
    : Unio (device_addr, period_us)
  {
    const char *help =
      "Available Commands:\n"
      "rd  <addr> [<type> [<base>]] (print <type> at <addr> in <base>)\n"
      "wr  <addr>  <type> <value>   (write <type> <value> at addr)\n"
      "            (<type>={c|b|w|l}, <base>={2|8|10|16})\n"
      "lb          (load binary file)\n"
      "hd [col=16] (hex memory dump)\n"
      "bp [<byte>] (bus period 10-100 [us])\n"
      "ea          (erase all)\n"
      "sa          (set all)\n"
      "ew          (enable write)\n"
      "dw          (disable write)\n"
      "sb          (standby)\n"
      "rs          (read status)\n"
      "ws <byte>   (write status)\n";

    byte mem[128];

    delay (1000);

    // PC: Give the user 60 seconds to upload a dump file.
    Serial.setTimeout (60000);

    Serial.println ("UNIO Shell by Pasqo\n");

    Serial.print (help);

    while (true)
    {
      Serial.print ("$ ");

      Tokenize ();

      Serial.print (m_tokv[0]);
      for (byte i = 1; i < m_tokc; i++)
      {
	Serial.print (' ');
	Serial.print (m_tokv[i]);
      }
      Serial.print ('\n');

      if (!strcmp (m_line, "help"))
      {
	Serial.println (help);
      }
      else if (!strcmp (m_line, "sb"))
      {
	StandBy ();
      }
      else if (!strcmp (m_line, "rs"))
      {
	byte st;
	if (ReadStatus (&st))
	{
	  Serial.print ( "BP1="); Serial.print   (bool(st & 1<<RDSR_BP1));
	  Serial.print (" BP0="); Serial.print   (bool(st & 1<<RDSR_BP0));
	  Serial.print (" WEL="); Serial.print   (bool(st & 1<<RDSR_WEL));
	  Serial.print (" WIP="); Serial.println (bool(st & 1<<RDSR_WIP));
	}
      }
      else if (!strcmp (m_line, "rd") && m_tokc)
      {
	word addr = atoi (m_tokv[1]);
	char type = m_tokc > 2 ? *m_tokv[2] : 'b';
	byte base = m_tokc > 3 ? atoi(m_tokv[3]) : type == 'c' ? 1 : 10;
	PrintMem (addr, type, base);
      }
      else if (!strcmp (m_line, "wr") && m_tokc == 4)
      {
	word addr = atoi (m_tokv[1]);
	char type = *m_tokv[2];
	WriteMem (addr, type, m_tokv[3]);
      }
      else if (!strcmp (m_line, "hd"))
      {
	byte cols = m_tokc == 2 ? atoi (m_tokv[1]) : 16;
	if (Read (mem, 0, 128))
	  HexDump (mem, 0, 128, cols);
      }
      else if (!strcmp (m_line, "lb"))
      {
	Serial.println ("please send a binary file within 60 seconds");
	word size = Serial.readBytes ((char*)mem, 128);
	if (size == 128)
	  Write (mem, 0, 128);
	else
	  Serial.println ("timed out");
      }
      else if (!strcmp (m_line, "bp") && m_tokc < 3)
      {
	if (m_tokc == 2)
	{
	  byte period_us = atoi (m_tokv[1]);
	  BusPeriod (period_us);
	}
	Serial.println (BusPeriod());
      }
      else if (!strcmp (m_line, "ea"))
      {
	EraseAll ();
      }
      else if (!strcmp (m_line, "sa"))
      {
	SetAll ();
      }
      else if (!strcmp (m_line, "ew"))
      {
	EnableWrite ();
      }
      else if (!strcmp (m_line, "dw"))
      {
	DisableWrite ();
      }
      else
      {
	Serial.println ("-E-: unknown or wrong command");
      }
      if (IsIdle())
      {
	Serial.println ("fail");
	StandBy ();
      }
    }
  }

  private:

  void Tokenize ()
  {
    char *c, *e = m_line + 63;
    m_tokc = 0;
    m_tokv[m_tokc] = c = m_line;
    while (c != e)
    {
      if (Serial.available())
      {
	*c = Serial.read();
	if (*c == '\n')
	  break;
	if (*c == ' ')
	{
	  *c = '\0';
	  m_tokv[++m_tokc] = c + 1;
	}
	c++;
      }
    }
    *c = '\0';
    m_tokc++;
  }

  union What
  {
    char *c;
    byte *b;
    word *w;
    long *l;
  };

  void HexDump (byte *mem, word addr, word len, byte cols = 16)
  {
    char s[4], *c;
    byte l, x;

    while (len)
    {
      l = x = len < cols ? len : cols;
      sprintf (s, "%02X:", addr);
      Serial.print (s);
      c = reinterpret_cast<char*>(mem);
      while (l--)
      {
	sprintf (s, " %02X", *mem++);
	Serial.print (s);
      }
      Serial.print (' ');
      l = x;
      while (l--)
	Serial.print (CharDump(*c++));
      addr += cols;
      len -= x;
      Serial.print ('\n');
    }
  }

  char CharDump (char c)
  {
    return c < 0x7F && c > 0x20 ? c : 0x2E;
  }

  void PrintMem (word addr, char type, byte base = 10)
  {
    What u;
    u.c = m_line;

    // For chars base is the len.
    // We can read a max of 64 chars in our buffer.
    if (Read ((byte*)m_line, addr, base))
    {
      switch (type)
      {
	case 'c': while (base--) Serial.print (CharDump(*u.c++)); break;
	case 'b': Serial.print (*u.b, base); break;
	case 'w': Serial.print (*u.w, base); break;
	case 'l': Serial.print (*u.l, base); break;
      }
      Serial.print ('\n');
    }
  }

  void WriteMem (word addr, char type, char *str)
  {
    long val = atol (str);
    switch (type)
    {
      case 'c': while (*str) Write ((byte*)str++, addr++, 1); break;
      case 'b': Write ((byte*)&val, addr, 1); break;
      case 'w': Write ((byte*)&val, addr, 2); break;
      case 'l': Write ((byte*)&val, addr, 4); break;
    }
  }
  private:

  char m_line[32], *m_tokv[4];
  byte m_tokc;

};
#endif // UnioShell_h
