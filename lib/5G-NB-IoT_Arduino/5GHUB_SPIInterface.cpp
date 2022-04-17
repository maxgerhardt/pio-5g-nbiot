/*
 * A library for 5G-NB-IoT Development board
 * This file is about the SPI Interface
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
 
#include <5GHUB_SPIInterface.h>
#include <Arduino.h>

_5GHUB_SPIInterface ::_5GHUB_SPIInterface (int8_t cspin, uint32_t freq,
                                       BitOrder dataOrder, uint8_t dataMode,
                                       SPIClass *theSPI) 
									   {
  _cs = cspin;
  _sck = _mosi = _miso = -1;
  _spi = theSPI;
  _begun = false;
  _spiSetting = new SPISettings(freq, dataOrder, dataMode);
  _freq = freq;
  _dataOrder = dataOrder;
  _dataMode = dataMode;
}

_5GHUB_SPIInterface ::_5GHUB_SPIInterface (int8_t cspin, int8_t sckpin,
                                       int8_t misopin, int8_t mosipin,
                                       uint32_t freq, BitOrder dataOrder,
                                       uint8_t dataMode) 								   
{
  _cs = cspin;
  _sck = sckpin;
  _miso = misopin;
  _mosi = mosipin;

#ifdef BUSIO_USE_FAST_PINIO
  csPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(cspin));
  csPinMask = digitalPinToBitMask(cspin);
  if (mosipin != -1) 
  {
    mosiPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(mosipin));
    mosiPinMask = digitalPinToBitMask(mosipin);
  }
  if (misopin != -1) 
  {
    misoPort = (BusIO_PortReg *)portInputRegister(digitalPinToPort(misopin));
    misoPinMask = digitalPinToBitMask(misopin);
  }
  clkPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(sckpin));
  clkPinMask = digitalPinToBitMask(sckpin);
#endif

  _freq = freq;
  _dataOrder = dataOrder;
  _dataMode = dataMode;
  _begun = false;
  _spiSetting = new SPISettings(freq, dataOrder, dataMode);
  _spi = NULL;
}

_5GHUB_SPIInterface ::~_5GHUB_SPIInterface () 
{
  if (_spiSetting) {
    delete _spiSetting;
    _spiSetting = nullptr;
  }
}

bool _5GHUB_SPIInterface ::begin(void) 
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_spi) 
  { // hardware SPI
    _spi->begin();
  } 
  else 
  {
    pinMode(_sck, OUTPUT);

    if ((_dataMode == SPI_MODE0) || (_dataMode == SPI_MODE1)) 
	{
      // idle low on mode 0 and 1
      digitalWrite(_sck, LOW);
    } else {
      // idle high on mode 2 or 3
      digitalWrite(_sck, HIGH);
    }
    if (_mosi != -1) 
	{
      pinMode(_mosi, OUTPUT);
      digitalWrite(_mosi, HIGH);
    }
    if (_miso != -1) 
	{
      pinMode(_miso, INPUT);
    }
  }

  _begun = true;
  return true;
}

void _5GHUB_SPIInterface ::transfer(uint8_t *buffer, size_t len) 
{
  if (_spi) 
  {
    // hardware SPI is easy

#if defined(SPARK)
    _spi->transfer(buffer, buffer, len, NULL);
#elif defined(STM32)
    for (size_t i = 0; i < len; i++) {
      _spi->transfer(buffer[i]);
    }
#else
    _spi->transfer(buffer, len);
#endif
    return;
  }

  uint8_t startbit;
  if (_dataOrder == SPI_BITORDER_LSBFIRST) 
  {
    startbit = 0x1;
  }
  else 
  {
    startbit = 0x80;
  }

  bool towrite, lastmosi = !(buffer[0] & startbit);
  uint8_t bitdelay_us = (1000000 / _freq) / 2;

  // for softSPI we'll do it by hand
  for (size_t i = 0; i < len; i++) {
    // software SPI
    uint8_t reply = 0;
    uint8_t send = buffer[i];

    for (uint8_t b = startbit; b != 0;
         b = (_dataOrder == SPI_BITORDER_LSBFIRST) ? b << 1 : b >> 1) 
		 {

      if (bitdelay_us) {
        delayMicroseconds(bitdelay_us);
      }

      if (_dataMode == SPI_MODE0 || _dataMode == SPI_MODE2) 
	  {
        towrite = send & b;
        if ((_mosi != -1) && (lastmosi != towrite)) 
		{
#ifdef BUSIO_USE_FAST_PINIO
          if (towrite)
            *mosiPort |= mosiPinMask;
          else
            *mosiPort &= ~mosiPinMask;
#else
          digitalWrite(_mosi, towrite);
#endif
          lastmosi = towrite;
        }

#ifdef BUSIO_USE_FAST_PINIO
        *clkPort |= clkPinMask; // Clock high
#else
        digitalWrite(_sck, HIGH);
#endif

        if (bitdelay_us) 
		{
          delayMicroseconds(bitdelay_us);
        }

        if (_miso != -1) 
		{
#ifdef BUSIO_USE_FAST_PINIO
          if (*misoPort & misoPinMask) 
		  {
#else
          if (digitalRead(_miso)) 
		  {
#endif
            reply |= b;
          }
        }

#ifdef BUSIO_USE_FAST_PINIO
        *clkPort &= ~clkPinMask; // Clock low
#else
        digitalWrite(_sck, LOW);
#endif
      } 
	  else 
	  { 
		// if (_dataMode == SPI_MODE1 || _dataMode == SPI_MODE3)
#ifdef BUSIO_USE_FAST_PINIO
        *clkPort |= clkPinMask; // Clock high
#else
        digitalWrite(_sck, HIGH);
#endif
        if (bitdelay_us) 
		{
          delayMicroseconds(bitdelay_us);
        }

        if (_mosi != -1) 
		{
#ifdef BUSIO_USE_FAST_PINIO
          if (send & b)
            *mosiPort |= mosiPinMask;
          else
            *mosiPort &= ~mosiPinMask;
#else
          digitalWrite(_mosi, send & b);
#endif
        }

#ifdef BUSIO_USE_FAST_PINIO
        *clkPort &= ~clkPinMask; // Clock low
#else
        digitalWrite(_sck, LOW);
#endif

        if (_miso != -1) 
		{
#ifdef BUSIO_USE_FAST_PINIO
          if (*misoPort & misoPinMask) 
		  {
#else
          if (digitalRead(_miso)) 
		  {
#endif
            reply |= b;
          }
        }
      }
      if (_miso != -1) 
	  {
        buffer[i] = reply;
      }
    }
  }
  return;
}

uint8_t _5GHUB_SPIInterface ::transfer(uint8_t send)
 {
  uint8_t data = send;
  transfer(&data, 1);
  return data;
}

void _5GHUB_SPIInterface ::beginTransaction(void) 
{
  if (_spi) 
  {
    _spi->beginTransaction(*_spiSetting);
  }
}

void _5GHUB_SPIInterface ::endTransaction(void) 
{
  if (_spi) 
  {
    _spi->endTransaction();
  }
}

bool _5GHUB_SPIInterface ::write(uint8_t *buffer, size_t len,
                               uint8_t *prefix_buffer, size_t prefix_len)
							   {
  if (_spi)
  {
    _spi->beginTransaction(*_spiSetting);
  }

  digitalWrite(_cs, LOW);
  // do the writing
  for (size_t i = 0; i < prefix_len; i++) 
  {
    transfer(prefix_buffer[i]);
  }
  for (size_t i = 0; i < len; i++) 
  {
    transfer(buffer[i]);
  }
  digitalWrite(_cs, HIGH);

  if (_spi) {
    _spi->endTransaction();
  }

  return true;
}

bool _5GHUB_SPIInterface ::read(uint8_t *buffer, size_t len, uint8_t sendvalue) 
{
  memset(buffer, sendvalue, len); // clear out existing buffer
  if (_spi) 
  {
    _spi->beginTransaction(*_spiSetting);
  }
  digitalWrite(_cs, LOW);
  transfer(buffer, len);
  digitalWrite(_cs, HIGH);

  if (_spi) 
  {
    _spi->endTransaction();
  }

  return true;
}

bool _5GHUB_SPIInterface ::write_then_read(uint8_t *write_buffer,
                                         size_t write_len, uint8_t *read_buffer,
                                         size_t read_len, uint8_t sendvalue) 
{
  if (_spi) 
  {
    _spi->beginTransaction(*_spiSetting);
  }

  digitalWrite(_cs, LOW);
  // do the writing
  for (size_t i = 0; i < write_len; i++) 
  {
    transfer(write_buffer[i]);
  }

  // do the reading
  for (size_t i = 0; i < read_len; i++) 
  {
    read_buffer[i] = transfer(sendvalue);
  }

  digitalWrite(_cs, HIGH);

  if (_spi) 
  {
    _spi->endTransaction();
  }

  return true;
}
