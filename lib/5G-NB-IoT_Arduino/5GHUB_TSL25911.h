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

#ifndef _TSL25911_H_
#define _TSL25911_H_

#include <Arduino.h>
#include <Wire.h>

//(channel 0) - (channel 1)
#define TSL25911_VISIBLE (2)

//channel 1
#define TSL25911_INFRARED (1)

#define TSL25911_FULLSPECTRUM (0)

//Default I2C address
#define TSL25911_ADDR (0x29)

//1010 0000: bits 7 and 5 for 'command normal'
#define TSL25911_COMMAND_BIT (0xA0)

//Special Function Command for "Clear ALS and no persist ALS interrupt"
#define TSL25911_CLEAR_INT (0xE7)

//Special Function Command for "Interrupt set - forces an interrupt"
#define TSL25911_TEST_INT (0xE4)

//1 = read/write word (rather than byte)
#define TSL25911_WORD_BIT (0x20)

//1 = using block read/write
#define TSL25911_BLOCK_BIT (0x10)

//Flag for ENABLE register to disable
#define TSL25911_ENABLE_POWEROFF (0x00)

//Flag for ENABLE register to enable
#define TSL25911_ENABLE_POWERON  (0x01)  

//ALS Enable. This field activates ALS function. Writing a one
//activates the ALS. Writing a zero disables the ALS.
#define TSL25911_ENABLE_AEN		 (0x02)  
										 
//ALS Interrupt Enable. When asserted permits ALS interrupts to be
//generated, subject to the persist filter.
#define TSL25911_ENABLE_AIEN	(0x10)  
										
//No Persist Interrupt Enable. When asserted NP Threshold conditions
//will generate an interrupt, bypassing the persist filter
#define TSL25911_ENABLE_NPIEN	(0x80)  
										
//Lux cooefficient
#define TSL25911_LUX_DF (408.0F)

//CH0 coefficient
#define TSL25911_LUX_COEFB (1.64F)

//CH1 coefficient A
#define TSL25911_LUX_COEFC (0.59F) 

//CH2 coefficient B
#define TSL25911_LUX_COEFD (0.86F)

// TSL259111 Register map
enum 
{
  TSL25911_REGISTER_ENABLE = 0x00,          // Enable register
  TSL25911_REGISTER_CONTROL = 0x01,         // Control register
  TSL25911_REGISTER_THRESHOLD_AILTL = 0x04, // ALS low threshold lower byte
  TSL25911_REGISTER_THRESHOLD_AILTH = 0x05, // ALS low threshold upper byte
  TSL25911_REGISTER_THRESHOLD_AIHTL = 0x06, // ALS high threshold lower byte
  TSL25911_REGISTER_THRESHOLD_AIHTH = 0x07, // ALS high threshold upper byte
  TSL25911_REGISTER_THRESHOLD_NPAILTL = 0x08, // No Persist ALS low threshold lower byte
  TSL25911_REGISTER_THRESHOLD_NPAILTH = 0x09, // No Persist ALS low threshold higher byte
  TSL25911_REGISTER_THRESHOLD_NPAIHTL = 0x0A, // No Persist ALS high threshold lower byte
  TSL25911_REGISTER_THRESHOLD_NPAIHTH = 0x0B, // No Persist ALS high threshold higher byte
  TSL25911_REGISTER_PERSIST_FILTER = 0x0C, // Interrupt persistence filter
  TSL25911_REGISTER_PACKAGE_PID = 0x11,    // Package Identification
  TSL25911_REGISTER_DEVICE_ID = 0x12,      // Device Identification
  TSL25911_REGISTER_DEVICE_STATUS = 0x13,  // Internal Status
  TSL25911_REGISTER_CHAN0_LOW = 0x14,      // Channel 0 data, low byte
  TSL25911_REGISTER_CHAN0_HIGH = 0x15,     // Channel 0 data, high byte
  TSL25911_REGISTER_CHAN1_LOW = 0x16,      // Channel 1 data, low byte
  TSL25911_REGISTER_CHAN1_HIGH = 0x17,     // Channel 1 data, high byte
};

/// Enumeration for the sensor integration timing
typedef enum 
{
  TSL25911_INTEGRATIONTIME_100MS = 0x00, // 100 millis
  TSL25911_INTEGRATIONTIME_200MS = 0x01, // 200 millis
  TSL25911_INTEGRATIONTIME_300MS = 0x02, // 300 millis
  TSL25911_INTEGRATIONTIME_400MS = 0x03, // 400 millis
  TSL25911_INTEGRATIONTIME_500MS = 0x04, // 500 millis
  TSL25911_INTEGRATIONTIME_600MS = 0x05, // 600 millis
} 
tsl25911IntegrationTime_t;

/// Enumeration for the persistance filter (for interrupts)
typedef enum {
  //  bit 7:4: 0
  TSL25911_PERSIST_EVERY = 0x00, // Every ALS cycle generates an interrupt
  TSL25911_PERSIST_ANY = 0x01,   // Any value outside of threshold range
  TSL25911_PERSIST_2 = 0x02,     // 2 consecutive values out of range
  TSL25911_PERSIST_3 = 0x03,     // 3 consecutive values out of range
  TSL25911_PERSIST_5 = 0x04,     // 5 consecutive values out of range
  TSL25911_PERSIST_10 = 0x05,    // 10 consecutive values out of range
  TSL25911_PERSIST_15 = 0x06,    // 15 consecutive values out of range
  TSL25911_PERSIST_20 = 0x07,    // 20 consecutive values out of range
  TSL25911_PERSIST_25 = 0x08,    // 25 consecutive values out of range
  TSL25911_PERSIST_30 = 0x09,    // 30 consecutive values out of range
  TSL25911_PERSIST_35 = 0x0A,    // 35 consecutive values out of range
  TSL25911_PERSIST_40 = 0x0B,    // 40 consecutive values out of range
  TSL25911_PERSIST_45 = 0x0C,    // 45 consecutive values out of range
  TSL25911_PERSIST_50 = 0x0D,    // 50 consecutive values out of range
  TSL25911_PERSIST_55 = 0x0E,    // 55 consecutive values out of range
  TSL25911_PERSIST_60 = 0x0F,    // 60 consecutive values out of range
} 
tsl25911Persist_t;

// Enumeration for the sensor gain
typedef enum 
{
  TSL25911_GAIN_LOW = 0x00,  // low gain (1x)
  TSL25911_GAIN_MED = 0x10,  // medium gain (25x)
  TSL25911_GAIN_HIGH = 0x20, // medium gain (428x)
  TSL25911_GAIN_MAX = 0x30,  // max gain (9876x)
} 
tsl25911Gain_t;

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with TSL259111
   Light Sensor
*/
/**************************************************************************/
class _5GHUB_TSL25911 : public _5GHUB_SensorInterface 
{
public:
  _5GHUB_TSL25911(int32_t sensorID = -1);

  boolean Begin(TwoWire *theWire, uint8_t addr = TSL25911_ADDR);
  boolean Begin(uint8_t addr = TSL25911_ADDR);
  void Enable(void);
  void Disable(void);

  float CalculateLux(uint16_t ch0, uint16_t ch1);
  void SetGain(tsl25911Gain_t gain);
  void SetTiming(tsl25911IntegrationTime_t integration);
  uint16_t GetLuminosity(uint8_t channel);
  uint32_t GetFullLuminosity();

  tsl25911IntegrationTime_t GetTiming();
  tsl25911Gain_t GetGain();

  // Interrupt
  void ClearInterrupt(void);
  void RegisterInterrupt(uint16_t lowerThreshold, uint16_t upperThreshold, tsl25911Persist_t persist);
  uint8_t GetStatus();

  void GetSensor(sensor_t *);

  bool GetEvent(sensors_event_t*) {return false;}
  void EnableAutoRange(bool enabled) { (void)enabled; };  
 
  
private:
  TwoWire *_i2c;

  void Write8(uint8_t r);
  void Write8(uint8_t r, uint8_t v);
  uint16_t Read16(uint8_t reg);
  uint8_t Read8(uint8_t reg);

  tsl25911IntegrationTime_t _integration;
  tsl25911Gain_t _gain;
  int32_t _sensorID;
  uint8_t _addr;

  boolean _initialized;
};
#endif
