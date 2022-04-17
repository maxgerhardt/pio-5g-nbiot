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

#ifndef _5GHUB_SPIINTERFACE_H_
#define _5GHUB_SPIINTERFACE_H_

#include <Arduino.h>
#include <SPI.h>

// some modern SPI definitions don't have BitOrder enum
#if (defined(__AVR__) && !defined(ARDUINO_ARCH_MEGAAVR)) ||                    \
    defined(ESP8266) || defined(TEENSYDUINO) || defined(SPARK) ||              \
    defined(ARDUINO_ARCH_SPRESENSE) || defined(MEGATINYCORE) ||                \
    defined(DXCORE) || defined(ARDUINO_AVR_ATmega4809) ||                      \
    defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209) ||      \
    defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609) ||      \
    defined(ARDUINO_AVR_ATmega1608) || defined(ARDUINO_AVR_ATmega809) ||       \
    defined(ARDUINO_AVR_ATmega808)
typedef enum _BitOrder 
{
  SPI_BITORDER_MSBFIRST = MSBFIRST,
  SPI_BITORDER_LSBFIRST = LSBFIRST,
} BitOrder;

#elif defined(ESP32) || defined(__ASR6501__)

// some modern SPI definitions don't have BitOrder enum and have different SPI
// mode defines
typedef enum _BitOrder 
{
  SPI_BITORDER_MSBFIRST = SPI_MSBFIRST,
  SPI_BITORDER_LSBFIRST = SPI_LSBFIRST,
} BitOrder;

#else
// Some platforms have a BitOrder enum but its named MSBFIRST/LSBFIRST
#define SPI_BITORDER_MSBFIRST MSBFIRST
#define SPI_BITORDER_LSBFIRST LSBFIRST
#endif

#if defined(__AVR__) || defined(TEENSYDUINO)
typedef volatile uint8_t BusIO_PortReg;
typedef uint8_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif defined(ESP8266) || defined(ESP32) || defined(__SAM3X8E__) ||            \
    defined(ARDUINO_ARCH_SAMD)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) &&                      \
    !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_RP2040)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#if not defined(__ASR6501__)
#define BUSIO_USE_FAST_PINIO
#endif

#else
#undef BUSIO_USE_FAST_PINIO
#endif

/**! The class which defines how we will talk to this device over SPI **/
class _5GHUB_SPIInterface 
{
public:
  _5GHUB_SPIInterface(int8_t cspin, uint32_t freq = 1000000,
                     BitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0, SPIClass *theSPI = &SPI);

  _5GHUB_SPIInterface(int8_t cspin, int8_t sck, int8_t miso, int8_t mosi,
                     uint32_t freq = 1000000,
                     BitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0);
  ~_5GHUB_SPIInterface();

  bool begin(void);
  bool read(uint8_t *buffer, size_t len, uint8_t sendvalue = 0xFF);
  bool write(uint8_t *buffer, size_t len, uint8_t *prefix_buffer = NULL,
             size_t prefix_len = 0);
  bool write_then_read(uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       uint8_t sendvalue = 0xFF);

  uint8_t transfer(uint8_t send);
  void transfer(uint8_t *buffer, size_t len);
  void beginTransaction(void);
  void endTransaction(void);

private:
  SPIClass *_spi;
  SPISettings *_spiSetting;
  uint32_t _freq;
  BitOrder _dataOrder;
  uint8_t _dataMode;

  int8_t _cs, _sck, _mosi, _miso;
  
#ifdef BUSIO_USE_FAST_PINIO
  BusIO_PortReg *mosiPort, *clkPort, *misoPort, *csPort;
  BusIO_PortMask mosiPinMask, misoPinMask, clkPinMask, csPinMask;
#endif

  
  bool _begun;
};

#endif
