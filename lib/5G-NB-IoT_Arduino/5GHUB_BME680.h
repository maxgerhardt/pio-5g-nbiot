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

#ifndef ___5GHUB_BME680_H__
#define ___5GHUB_BME680_H__

#include <5GHUB_I2CInterface.h>
#include <5GHUB_SPIInterface.h>
#include "bme68x.h"

#define BME68X_DEFAULT_ADDRESS (0x77)    // The default I2C address
#define BME68X_DEFAULT_SPIFREQ (1000000) // The default SPI Clock speed

#define BME680_OS_16X 		BME68X_OS_16X
#define BME680_OS_8X 		BME68X_OS_8X
#define BME680_OS_4X 		BME68X_OS_4X
#define BME680_OS_2X 		BME68X_OS_2X
#define BME680_OS_1X 		BME68X_OS_1X
#define BME680_OS_NONE 		BME68X_OS_NONE

#define BME680_FILTER_SIZE_127		BME68X_FILTER_SIZE_127 
#define BME680_FILTER_SIZE_63		BME68X_FILTER_SIZE_63 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_31		BME68X_FILTER_SIZE_31 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_15		BME68X_FILTER_SIZE_15 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_7		BME68X_FILTER_SIZE_7 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_3		BME68X_FILTER_SIZE_3 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_1		BME68X_FILTER_SIZE_1 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_0		BME68X_FILTER_OFF ///< Alias for BME680 existing examples

class _5GHUB_BME680 
{
public:
  // Value returned by remainingReadingMillis indicating no asynchronous reading has been initiated by BeginReading
  static constexpr int reading_not_started = -1;
  // Value returned by remainingReadingMillis indicating asynchronous reading is complete and calling EndReading will not block.
  static constexpr int reading_complete = 0;

  _5GHUB_BME680(TwoWire *theWire = &Wire);
  _5GHUB_BME680(int8_t cspin, SPIClass *theSPI = &SPI);
  _5GHUB_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

  bool Begin(uint8_t addr = BME68X_DEFAULT_ADDRESS, bool initSettings = true);
  float ReadTemperature();
  float ReadPressure();
  float ReadHumidity();
  uint32_t ReadGas();
  float ReadAltitude(float seaLevel);

  bool SetTemperatureOversampling(uint8_t os);
  bool SetPressureOversampling(uint8_t os);
  bool SetHumidityOversampling(uint8_t os);
  bool SetIIRFilterSize(uint8_t fs);
  bool SetGasHeater(uint16_t heaterTemp, uint16_t heaterTime);
  bool SetODR(uint8_t odr);

  // Perform a reading in blocking mode.
  bool PerformReading();

  uint32_t BeginReading();

  bool EndReading();

  int RemainingReadingMillis();

  // Temperature (Celsius) assigned after calling performReading() or EndReading()
  float temperature;
  // Pressure (Pascals) assigned after calling performReading() or EndReading()
  uint32_t pressure;
  // Humidity (RH %) assigned after calling performReading() or EndReading()
  float humidity;
  
  //Gas resistor (ohms) assigned after calling performReading() or EndReading()
  uint32_t gas_resistance;

private:
  _5GHUB_I2CInterface *_i2cIntrfc = NULL;
  _5GHUB_SPIInterface *_spiIntrfc = NULL;  
  TwoWire *_wire = NULL;

  int32_t _sensorID;
  uint32_t _meas_start = 0;
  uint16_t _meas_period = 0;

  struct bme68x_dev gas_sensor;
  struct bme68x_conf gas_conf;
  struct bme68x_heatr_conf gas_heatr_conf;
};

#endif
