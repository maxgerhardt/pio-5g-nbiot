/*
 * A library for 5G-NB-IoT Development board
 * This file is about the I2C Interface
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
 
#ifndef _5GHUB_I2INTERFACE_H_
#define _5GHUB_I2INTERFACE_H_

#include <Arduino.h>
#include <Wire.h>

class _5GHUB_I2CInterface 
{
public:
  _5GHUB_I2CInterface(uint8_t addr, TwoWire *theWire = &Wire);
  uint8_t address(void);
  bool begin(bool addr_detect = true);
  bool detected(void);

  bool read(uint8_t *buffer, size_t len, bool stop = true);
  bool write(const uint8_t *buffer, size_t len, bool stop = true, const uint8_t *prefix_buffer = NULL, size_t prefix_len = 0);
  bool write_then_read(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len, bool stop = false);
  bool setSpeed(uint32_t desiredclk);

  size_t maxBufferSize() { return _maxBufferSize; }

private:
  uint8_t _addr;
  TwoWire *_wire;
  bool _begun;
  size_t _maxBufferSize;
};

#endif