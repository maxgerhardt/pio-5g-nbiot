/*
 * A library for 5G-NB-IoT Development board
 * This file is about the BME680 sensor interface 
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

#include "5GHUB_BME680.h"
#include "Arduino.h"

// I2C and SPI interface functions
//
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *interface);
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *interface);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *interface);
static void delay_usec(uint32_t us, void *intf_ptr);

_5GHUB_BME680::_5GHUB_BME680(TwoWire *theWire) : _meas_start(0), _meas_period(0) 
{
  _wire = theWire;
}

_5GHUB_BME680::_5GHUB_BME680(int8_t cspin, SPIClass *theSPI) : _meas_start(0), _meas_period(0) 
{
  _spiIntrfc = new _5GHUB_SPIInterface(cspin, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE0, theSPI);
}

_5GHUB_BME680::_5GHUB_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin) : _meas_start(0), _meas_period(0) 
{
  _spiIntrfc = new _5GHUB_SPIInterface(cspin, sckpin, misopin, mosipin, 1000000,
                                   SPI_BITORDER_MSBFIRST, SPI_MODE0);
}

bool _5GHUB_BME680::Begin(uint8_t addr, bool initSettings) 
{
  int8_t rslt;

  if (!_spiIntrfc) 
  {
    if (_i2cIntrfc) 
	{
      delete _i2cIntrfc;
    }
    _i2cIntrfc = new _5GHUB_I2CInterface(addr, _wire);
    if (!_i2cIntrfc->begin()) 
	{
      return false;
    }

    gas_sensor.chip_id = addr;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = (void *)_i2cIntrfc;
    gas_sensor.read = &i2c_read;
    gas_sensor.write = &i2c_write;

  } 
  else 
  {
    if (!_spiIntrfc->begin()) 
	{
      return false;
    }

    gas_sensor.chip_id = 0;
    gas_sensor.intf = BME68X_SPI_INTF;
    gas_sensor.intf_ptr = (void *)_spiIntrfc;
    gas_sensor.read = &spi_read;
    gas_sensor.write = &spi_write;
  }

	// The ambient temperature in d	eg C is used for defining the heater temperature
  gas_sensor.amb_temp = 25;
  gas_sensor.delay_us = delay_usec;

  rslt = bme68x_init(&gas_sensor);

  if (rslt != BME68X_OK)
    return false;

  if (initSettings) 
  {
    SetIIRFilterSize(BME68X_FILTER_SIZE_3);
    SetODR(BME68X_ODR_NONE);
    SetHumidityOversampling(BME68X_OS_2X);
    SetPressureOversampling(BME68X_OS_4X);
    SetTemperatureOversampling(BME68X_OS_8X);
    // 320*C for 150 ms
	SetGasHeater(320, 150);
  } 
  else 
  {
    SetGasHeater(0, 0);
  }
  // don't do anything till we request a reading
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);

  if (rslt != BME68X_OK)
    return false;

  return true;
}

float _5GHUB_BME680::ReadTemperature(void) 
{
  PerformReading();
  return temperature;
}

float _5GHUB_BME680::ReadPressure(void) 
{
  PerformReading();
  return pressure;
}

float _5GHUB_BME680::ReadHumidity(void) 
{
  PerformReading();
  return humidity;
}

uint32_t _5GHUB_BME680::ReadGas(void) 
{
  PerformReading();
  return gas_resistance;
}

float _5GHUB_BME680::ReadAltitude(float seaLevel) 
{
  // Equation taken from BMP180 datasheet:
  // using other equation can give unstable results

  float atmospheric = ReadPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

bool _5GHUB_BME680::PerformReading(void) 
{ 
	return EndReading(); 
}

uint32_t _5GHUB_BME680::BeginReading(void) 
{
  if (_meas_start != 0)
  {
    // A measurement is already in progress
    return _meas_start + _meas_period;
  }

  int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);

  if (rslt != BME68X_OK)
    return false;

  //Calculate delay period in microseconds
  uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor) + ((uint32_t)gas_heatr_conf.heatr_dur * 1000);

  _meas_start = millis();
  _meas_period = delayus_period / 1000;

  return _meas_start + _meas_period;
}

bool _5GHUB_BME680::EndReading(void) 
{
  uint32_t meas_end = BeginReading();

  if (meas_end == 0) 
  {
    return false;
  }

  int remaining_millis = RemainingReadingMillis();

  if (remaining_millis > 0) 
  {
	//Delay till the measurement is ready
    delay(static_cast<unsigned int>(remaining_millis) * 2);
  }
  
  // Allow new measurement to begin
  _meas_start = 0;
  _meas_period = 0;

  struct bme68x_data data;
  uint8_t n_fields;

  int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &gas_sensor);

  if (rslt != BME68X_OK)
    return false;

  if (n_fields) 
  {
    temperature = data.temperature;
    humidity = data.humidity;
    pressure = data.pressure;

    if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) 
	{
      gas_resistance = data.gas_resistance;
    } 
	else 
	{
      gas_resistance = 0;
    }
  }

  return true;
}

int _5GHUB_BME680::RemainingReadingMillis(void) 
{
  if (_meas_start != 0) 
  {
    //A measurement is already in progress
    int remaining_time = (int)_meas_period - (millis() - _meas_start);
    return remaining_time < 0 ? reading_complete : remaining_time;
  }
  return reading_not_started;
}

bool _5GHUB_BME680::SetGasHeater(uint16_t heaterTemp, uint16_t heaterTime) 
{

  if ((heaterTemp == 0) || (heaterTime == 0)) 
  {
    gas_heatr_conf.enable = BME68X_DISABLE;
  } 
  else 
  {
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp = heaterTemp;
    gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);

  return rslt == 0;
}

bool _5GHUB_BME680::SetODR(uint8_t odr) 
{
  if (odr > BME68X_ODR_NONE)
    return false;

  gas_conf.odr = odr;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);

  return rslt == 0;
}

bool _5GHUB_BME680::SetTemperatureOversampling(uint8_t oversample) 
{
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);

  return rslt == 0;
}

bool _5GHUB_BME680::SetHumidityOversampling(uint8_t oversample) 
{
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);

  return rslt == 0;
}

bool _5GHUB_BME680::SetPressureOversampling(uint8_t oversample) 
{
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

bool _5GHUB_BME680::SetIIRFilterSize(uint8_t filtersize) 
{
  if (filtersize > BME68X_FILTER_SIZE_127)
    return false;
  gas_conf.filter = filtersize;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf) 
{

  _5GHUB_I2CInterface *_dev = (_5GHUB_I2CInterface *)intf;

  if (!_dev->write_then_read(&reg_addr, 1, reg_data, len, true)) 
  {
    return -1;
  }

  return 0;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf) 
{
  _5GHUB_I2CInterface *_dev = (_5GHUB_I2CInterface *)intf;

  if (!_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1)) 
  {
    return -1;
  }
  return 0;
}

static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) 
{
  _5GHUB_SPIInterface *_dev = (_5GHUB_SPIInterface *)intf_ptr;

  reg_addr |= 0x80;

  if (!_dev->write_then_read(&reg_addr, 1, reg_data, len, 0x0)) 
  {
    return -1;
  }

  return 0;
}

static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) 
{
  _5GHUB_SPIInterface *_dev = (_5GHUB_SPIInterface *)intf_ptr;

  if (!_dev->write((uint8_t *)reg_data, len, &reg_addr, 1)) 
  {
    return -1;
  }

  return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) 
{
  delayMicroseconds(us);
  yield();
}
