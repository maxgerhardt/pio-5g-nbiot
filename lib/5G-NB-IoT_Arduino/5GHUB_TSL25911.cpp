/*
 * A library for 5G_NB_IoT Development board
 * This file is about the TSL25911 light sensor common function
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

#include "5GHUB_Sensor.h"
#include "5GHUB_TSL25911.h"
#include <stdlib.h>

_5GHUB_TSL25911::_5GHUB_TSL25911(int32_t sensorID)
 {
  _initialized = false;
  _integration = TSL25911_INTEGRATIONTIME_100MS;
  _gain = TSL25911_GAIN_MED;
  _sensorID = sensorID;
}

boolean _5GHUB_TSL25911::Begin(TwoWire *theWire, uint8_t addr) 
{
  _i2c = theWire;
  _i2c->begin();
  _addr = addr;

  uint8_t id = Read8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_DEVICE_ID);
  if (id != 0x50) 
  {
    return false;
  }
  _initialized = true;

  // Set default integration time and gain
  SetTiming(_integration);
  SetGain(_gain);

  // Note: by default, the device is in power down mode on bootup
  Disable();

  return true;
}

boolean _5GHUB_TSL25911::Begin(uint8_t addr) 
{ 
	return Begin(&Wire, addr); 
}

void _5GHUB_TSL25911::Enable(void) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  // Enable the device by setting the control bit to 0x01
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_ENABLE, TSL25911_ENABLE_POWERON | TSL25911_ENABLE_AEN | TSL25911_ENABLE_AIEN | TSL25911_ENABLE_NPIEN);
}

void _5GHUB_TSL25911::Disable(void) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  // Disable the device by setting the control bit to 0x00
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_ENABLE, TSL25911_ENABLE_POWEROFF);
}

void _5GHUB_TSL25911::SetGain(tsl25911Gain_t gain) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  Enable();
  _gain = gain;
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_CONTROL, _integration | _gain);
  Disable();
}

tsl25911Gain_t _5GHUB_TSL25911::GetGain() 
{ 
	return _gain; 
}

void _5GHUB_TSL25911::SetTiming(tsl25911IntegrationTime_t integration) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  Enable();
  _integration = integration;
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_CONTROL, _integration | _gain);
  Disable();
}

tsl25911IntegrationTime_t _5GHUB_TSL25911::GetTiming() 
{ 
	return _integration; 
}

float _5GHUB_TSL25911::CalculateLux(uint16_t ch0, uint16_t ch1) 
{
  float atime, again;
  float cpl, lux1, lux2, lux;
  uint32_t chan0, chan1;

  // Check for overflow conditions first
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF)) 
  {
    // Signal an overflow
    return -1;
  }

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (_integration) 
  {
  case TSL25911_INTEGRATIONTIME_100MS:
    atime = 100.0F;
    break;
  case TSL25911_INTEGRATIONTIME_200MS:
    atime = 200.0F;
    break;
  case TSL25911_INTEGRATIONTIME_300MS:
    atime = 300.0F;
    break;
  case TSL25911_INTEGRATIONTIME_400MS:
    atime = 400.0F;
    break;
  case TSL25911_INTEGRATIONTIME_500MS:
    atime = 500.0F;
    break;
  case TSL25911_INTEGRATIONTIME_600MS:
    atime = 600.0F;
    break;
  default: // 100ms
    atime = 100.0F;
    break;
  }

  switch (_gain) 
  {
  case TSL25911_GAIN_LOW:
    again = 1.0F;
    break;
  case TSL25911_GAIN_MED:
    again = 25.0F;
    break;
  case TSL25911_GAIN_HIGH:
    again = 428.0F;
    break;
  case TSL25911_GAIN_MAX:
    again = 9876.0F;
    break;
  default:
    again = 1.0F;
    break;
  }

  cpl = (atime * again) / TSL25911_LUX_DF;
  lux = (((float)ch0 - (float)ch1)) * (1.0F - ((float)ch1 / (float)ch0)) / cpl;

  return lux;
}

uint32_t _5GHUB_TSL25911::GetFullLuminosity(void) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return 0;
    }
  }

  // Enable the device
  Enable();

  // Wait x ms for ADC to complete
  for (uint8_t d = 0; d <= _integration; d++) 
  {
    delay(120);
  }

  uint32_t x;
  uint16_t y;
  y = Read16(TSL25911_COMMAND_BIT | TSL25911_REGISTER_CHAN0_LOW);
  x = Read16(TSL25911_COMMAND_BIT | TSL25911_REGISTER_CHAN1_LOW);
  x <<= 16;
  x |= y;

  Disable();

  return x;
}

uint16_t _5GHUB_TSL25911::GetLuminosity(uint8_t channel) 
{
  uint32_t x = GetFullLuminosity();

  if (channel == TSL25911_FULLSPECTRUM) 
  {
    // Reads two byte value from channel 0 (visible + infrared)
    return (x & 0xFFFF);
  } else if (channel == TSL25911_INFRARED) 
  {
    // Reads two byte value from channel 1 (infrared)
    return (x >> 16);
  } else if (channel == TSL25911_VISIBLE) 
  {
    // Reads all and subtracts out just the visible!
    return ((x & 0xFFFF) - (x >> 16));
  }

  // unknown channel!
  return 0;
}

void _5GHUB_TSL25911::RegisterInterrupt( uint16_t lowerThreshold, uint16_t upperThreshold, tsl25911Persist_t persist = TSL25911_PERSIST_ANY) 
	{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  Enable();
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_PERSIST_FILTER, persist);
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_THRESHOLD_AILTL,
         lowerThreshold);
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_THRESHOLD_AILTH,
         lowerThreshold >> 8);
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_THRESHOLD_AIHTL,
         upperThreshold);
  Write8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_THRESHOLD_AIHTH,
         upperThreshold >> 8);
  Disable();
}

void _5GHUB_TSL25911::ClearInterrupt() 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return;
    }
  }

  Enable();
  Write8(TSL25911_CLEAR_INT);
  Disable();
}

uint8_t _5GHUB_TSL25911::GetStatus(void) 
{
  if (!_initialized) 
  {
    if (!Begin()) 
	{
      return 0;
    }
  }

  Enable();
  uint8_t x;
  x = Read8(TSL25911_COMMAND_BIT | TSL25911_REGISTER_DEVICE_STATUS);
  Disable();
  return x;
}

void _5GHUB_TSL25911::GetSensor(sensor_t *sensor) 
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "TSL25911", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_LIGHT;
  sensor->min_delay = 0;
  sensor->max_value = 88000.0;
  sensor->min_value = 0.0;
  sensor->resolution = 0.001;
}

uint8_t _5GHUB_TSL25911::Read8(uint8_t reg) 
{
  uint8_t x;

  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom(_addr, 1);
  x = _i2c->read();

  return x;
}

uint16_t _5GHUB_TSL25911::Read16(uint8_t reg) 
{
  uint16_t x;
  uint16_t t;

  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom(_addr, 2);
  t = _i2c->read();
  x = _i2c->read();

  x <<= 8;
  x |= t;
  return x;
}

void _5GHUB_TSL25911::Write8(uint8_t reg, uint8_t value) 
{
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->write(value);
  _i2c->endTransmission();
}

void _5GHUB_TSL25911::Write8(uint8_t reg) 
{
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->endTransmission();
}
