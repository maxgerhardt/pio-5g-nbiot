/*
 * A library for 5G-NB-IoT Development board
 * This file is about the BG96 AT Command list
 * 
 * Copyright (c) 
 * @Author       :
 * @Create time  :
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include <5GHUB_I2CInterface.h>
#include <Wire.h>

_5GHUB_I2CInterface ::_5GHUB_I2CInterface (uint8_t addr, TwoWire *theWire) 
{
  _addr = addr;
  _wire = theWire;
  _begun = false;
#ifdef ARDUINO_ARCH_SAMD
  _maxBufferSize = 250; // as defined in Wire.h's RingBuffer
#else
  _maxBufferSize = 32;
#endif
}

bool _5GHUB_I2CInterface ::begin(bool addr_detect) 
{
  _wire->begin();
  _begun = true;

  if (addr_detect) 
  {
    return detected();
  }
  return true;
}

bool _5GHUB_I2CInterface ::detected(void) 
{
  // Init I2C if not done yet
  if (!_begun && !begin()) 
  {
    return false;
  }

  // A basic scanner, see if it ACK's
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() == 0) 
  {
    return true;
  }
  return false;
}

bool _5GHUB_I2CInterface ::write(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len) 
							   {
  if ((len + prefix_len) > maxBufferSize()) 
  {
    return false;
  }

  _wire->beginTransmission(_addr);

  // Write the prefix data (usually an address)
  if ((prefix_len != 0) && (prefix_buffer != NULL)) 
  {
    if (_wire->write(prefix_buffer, prefix_len) != prefix_len) 
	{
      return false;
    }
  }

  // Write the data itself
  if (_wire->write(buffer, len) != len) 
  {
    return false;
  }



  if (_wire->endTransmission(stop) == 0) 
  {
    return true;
  } 
  else
  {
    return false;
  }
}

bool _5GHUB_I2CInterface ::read(uint8_t *buffer, size_t len, bool stop) 
{
  if (len > maxBufferSize()) 
  {
    return false;
  }

  size_t recv = _wire->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);
  if (recv != len) 
  {
    return false;
  }

  for (uint16_t i = 0; i < len; i++) 
  {
    buffer[i] = _wire->read();
  }

  return true;
}

bool _5GHUB_I2CInterface ::write_then_read(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len, bool stop) 
										 {
  if (!write(write_buffer, write_len, stop)) 
  {
    return false;
  }

  return read(read_buffer, read_len);
}

uint8_t _5GHUB_I2CInterface ::address(void) 
{ 
	return _addr; 
}

bool _5GHUB_I2CInterface ::setSpeed(uint32_t desiredclk) 
{
	#if (ARDUINO >= 157) && !defined(ARDUINO_STM32_FEATHER)
	  _wire->setClock(desiredclk);
	  return true;
	#else
	  return false;
	#endif
}
