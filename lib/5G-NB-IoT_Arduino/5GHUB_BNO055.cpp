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


#include "Arduino.h"
#include <limits.h>
#include <math.h>

#include "5GHUB_BNO055.h"


_5GHUB_BNO055::_5GHUB_BNO055(int32_t sensorID, uint8_t address,
                                 TwoWire *theWire) 
{
// BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  theWire->setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  _sensorID = sensorID;
  i2c_dev = new _5GHUB_I2CInterface(address, theWire);
}

bool _5GHUB_BNO055::Begin(_5GHUB_bno055_opmode_t mode) 
{

  if (!i2c_dev->begin()) {
    return false;
  }

  /* Make sure we have the right device */
  uint8_t id = Read8(BNO055_CHIP_ID_ADDR);
  if (id != BNO055_ID) {
    delay(1000); // hold on for boot
    id = Read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
      return false; // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  SetMode(OPERATION_MODE_CONFIG);

  /* Reset */
  Write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(30);
  while (Read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) 
  {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  Write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  Write8(BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  Write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  Write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  Write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  Write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(mode);
  delay(20);

  return true;
}

void _5GHUB_BNO055::SetMode(_5GHUB_bno055_opmode_t mode) 
{
  _mode = mode;
  Write8(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
}

void _5GHUB_BNO055::SetAxisRemap(
    _5GHUB_bno055_axis_remap_config_t remapcode) 
{
	_5GHUB_bno055_opmode_t modeback = _mode;

  SetMode(OPERATION_MODE_CONFIG);
  delay(25);
  Write8(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(modeback);
  delay(20);
}

void _5GHUB_BNO055::SetAxisSign(_5GHUB_bno055_axis_remap_sign_t remapsign) 
{
  _5GHUB_bno055_opmode_t modeback = _mode;

  SetMode(OPERATION_MODE_CONFIG);
  delay(25);
  Write8(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(modeback);
  delay(20);
}

void _5GHUB_BNO055::SetExtCrystalUse(boolean usextal) 
{
  _5GHUB_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  SetMode(OPERATION_MODE_CONFIG);
  delay(25);
  Write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) 
  {
    Write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else 
  {
    Write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(modeback);
  delay(20);
}

void _5GHUB_BNO055::GetSystemStatus(uint8_t *system_status,
                                      uint8_t *self_test_result,
                                      uint8_t *system_error) 
{
  Write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0)
    *system_status = Read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    *self_test_result = Read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    *system_error = Read8(BNO055_SYS_ERR_ADDR);

  delay(200);
}

void _5GHUB_BNO055::GetRevInfo(_5GHUB_bno055_rev_info_t *info) 
{
  uint8_t a, b;

  memset(info, 0, sizeof(_5GHUB_bno055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = Read8(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev = Read8(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev = Read8(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev = Read8(BNO055_BL_REV_ID_ADDR);

  a = Read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = Read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

void _5GHUB_BNO055::GetCalibration(uint8_t *sys, uint8_t *gyro,
                                     uint8_t *accel, uint8_t *mag) 
{
  uint8_t calData = Read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) 
  {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) 
  {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) 
  {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) 
  {
    *mag = calData & 0x03;
  }
}

int8_t _5GHUB_BNO055::GetTemp() 
{
  int8_t temp = (int8_t)(Read8(BNO055_TEMP_ADDR));
  return temp;
}

imu::Vector<3> _5GHUB_BNO055::GetVector(_5GHUB_vector_type_t vector_type) 
{
  imu::Vector<3> xyz;
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  ReadLen((_5GHUB_bno055_reg_t)vector_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type) {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  }

  return xyz;
}

imu::Quaternion _5GHUB_BNO055::GetQuat() 
{
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  ReadLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /*
   * Quaternion
   * See DATASHEET of BNO055 - 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  return quat;
}

void _5GHUB_BNO055::GetSensor(sensor_t *sensor) 
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ORIENTATION;
  sensor->min_delay = 0;
  sensor->max_value = 0.0F;
  sensor->min_value = 0.0F;
  sensor->resolution = 0.01F;
}

bool _5GHUB_BNO055::GetEvent(sensors_event_t *event) 
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ORIENTATION;
  event->timestamp = millis();

  /* Get a Euler angle sample for orientation */
  imu::Vector<3> euler = GetVector(_5GHUB_BNO055::VECTOR_EULER);
  event->orientation.x = euler.x();
  event->orientation.y = euler.y();
  event->orientation.z = euler.z();

  return true;
}

bool _5GHUB_BNO055::GetEvent(sensors_event_t *event,
                               _5GHUB_vector_type_t vec_type) 
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->timestamp = millis();

  // read the data according to vec_type
  imu::Vector<3> vec;
  if (vec_type == _5GHUB_BNO055::VECTOR_LINEARACCEL) 
  {
    event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
    vec = GetVector(_5GHUB_BNO055::VECTOR_LINEARACCEL);

    event->acceleration.x = vec.x();
    event->acceleration.y = vec.y();
    event->acceleration.z = vec.z();
  } else if (vec_type == _5GHUB_BNO055::VECTOR_ACCELEROMETER) 
  {
    event->type = SENSOR_TYPE_ACCELEROMETER;
    vec = GetVector(_5GHUB_BNO055::VECTOR_ACCELEROMETER);

    event->acceleration.x = vec.x();
    event->acceleration.y = vec.y();
    event->acceleration.z = vec.z();
  } else if (vec_type == _5GHUB_BNO055::VECTOR_GRAVITY) 
  {
    event->type = SENSOR_TYPE_GRAVITY;
    vec = GetVector(_5GHUB_BNO055::VECTOR_GRAVITY);

    event->acceleration.x = vec.x();
    event->acceleration.y = vec.y();
    event->acceleration.z = vec.z();
  } else if (vec_type == _5GHUB_BNO055::VECTOR_EULER) 
  {
    event->type = SENSOR_TYPE_ORIENTATION;
    vec = GetVector(_5GHUB_BNO055::VECTOR_EULER);

    event->orientation.x = vec.x();
    event->orientation.y = vec.y();
    event->orientation.z = vec.z();
  } else if (vec_type == _5GHUB_BNO055::VECTOR_GYROSCOPE) 
  {
    event->type = SENSOR_TYPE_GYROSCOPE;
    vec = GetVector(_5GHUB_BNO055::VECTOR_GYROSCOPE);

    event->gyro.x = vec.x() * SENSORS_DPS_TO_RADS;
    event->gyro.y = vec.y() * SENSORS_DPS_TO_RADS;
    event->gyro.z = vec.z() * SENSORS_DPS_TO_RADS;
  } else if (vec_type == _5GHUB_BNO055::VECTOR_MAGNETOMETER) 
  {
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    vec = GetVector(_5GHUB_BNO055::VECTOR_MAGNETOMETER);

    event->magnetic.x = vec.x();
    event->magnetic.y = vec.y();
    event->magnetic.z = vec.z();
  }

  return true;
}

bool _5GHUB_BNO055::GetSensorOffsets(uint8_t *calibData) 
{
  if (isFullyCalibrated()) 
  {
    _5GHUB_bno055_opmode_t lastMode = _mode;
    SetMode(OPERATION_MODE_CONFIG);

    ReadLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    SetMode(lastMode);
    return true;
  }
  return false;
}

bool _5GHUB_BNO055::GetSensorOffsets(
    _5GHUB_bno055_offsets_t &offsets_type) 
{
  if (isFullyCalibrated()) 
  {
    _5GHUB_bno055_opmode_t lastMode = _mode;
    SetMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Accel offset range depends on the G-range:
       +/-2g  = +/- 2000 mg
       +/-4g  = +/- 4000 mg
       +/-8g  = +/- 8000 mg
       +/-1§g = +/- 16000 mg */
    offsets_type.accel_offset_x = (Read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                  (Read8(ACCEL_OFFSET_X_LSB_ADDR));
    offsets_type.accel_offset_y = (Read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                  (Read8(ACCEL_OFFSET_Y_LSB_ADDR));
    offsets_type.accel_offset_z = (Read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                  (Read8(ACCEL_OFFSET_Z_LSB_ADDR));

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    offsets_type.mag_offset_x =
        (Read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (Read8(MAG_OFFSET_X_LSB_ADDR));
    offsets_type.mag_offset_y =
        (Read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (Read8(MAG_OFFSET_Y_LSB_ADDR));
    offsets_type.mag_offset_z =
        (Read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (Read8(MAG_OFFSET_Z_LSB_ADDR));

    /* Gyro offset range depends on the DPS range:
      2000 dps = +/- 32000 LSB
      1000 dps = +/- 16000 LSB
       500 dps = +/- 8000 LSB
       250 dps = +/- 4000 LSB
       125 dps = +/- 2000 LSB
       ... where 1 DPS = 16 LSB */
    offsets_type.gyro_offset_x =
        (Read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (Read8(GYRO_OFFSET_X_LSB_ADDR));
    offsets_type.gyro_offset_y =
        (Read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (Read8(GYRO_OFFSET_Y_LSB_ADDR));
    offsets_type.gyro_offset_z =
        (Read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (Read8(GYRO_OFFSET_Z_LSB_ADDR));

    /* Accelerometer radius = +/- 1000 LSB */
    offsets_type.accel_radius =
        (Read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (Read8(ACCEL_RADIUS_LSB_ADDR));

    /* Magnetometer radius = +/- 960 LSB */
    offsets_type.mag_radius =
        (Read8(MAG_RADIUS_MSB_ADDR) << 8) | (Read8(MAG_RADIUS_LSB_ADDR));

    SetMode(lastMode);
    return true;
  }
  return false;
}

void _5GHUB_BNO055::SetSensorOffsets(const uint8_t *calibData) 
{
  _5GHUB_bno055_opmode_t lastMode = _mode;
  SetMode(OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  /* A writeLen() would make this much cleaner */
  Write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
  Write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
  Write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
  Write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
  Write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
  Write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

  Write8(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
  Write8(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
  Write8(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
  Write8(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
  Write8(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
  Write8(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

  Write8(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
  Write8(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
  Write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
  Write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
  Write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
  Write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

  Write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
  Write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

  Write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
  Write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

  SetMode(lastMode);
}

void _5GHUB_BNO055::SetSensorOffsets(
    const _5GHUB_bno055_offsets_t &offsets_type) 
{
	  _5GHUB_bno055_opmode_t lastMode = _mode;
	  SetMode(OPERATION_MODE_CONFIG);
	  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  Write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
  Write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
  Write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
  Write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
  Write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
  Write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

  Write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
  Write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
  Write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
  Write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
  Write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
  Write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

  Write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
  Write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
  Write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
  Write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
  Write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
  Write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

  Write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
  Write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

  Write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
  Write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

  SetMode(lastMode);
}

bool _5GHUB_BNO055::isFullyCalibrated() 
{
  uint8_t system, gyro, accel, mag;
  GetCalibration(&system, &gyro, &accel, &mag);

  switch (_mode) {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

void _5GHUB_BNO055::EnterSuspendMode() 
{
  _5GHUB_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  SetMode(OPERATION_MODE_CONFIG);
  delay(25);
  Write8(BNO055_PWR_MODE_ADDR, 0x02);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(modeback);
  delay(20);
}

void _5GHUB_BNO055::EnterNormalMode() 
{
  _5GHUB_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  SetMode(OPERATION_MODE_CONFIG);
  delay(25);
  Write8(BNO055_PWR_MODE_ADDR, 0x00);
  /* Set the requested operating mode (see section 3.3) */
  SetMode(modeback);
  delay(20);
}

bool _5GHUB_BNO055::Write8(_5GHUB_bno055_reg_t reg, byte value) 
{
  uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
  return i2c_dev->write(buffer, 2);
}

byte _5GHUB_BNO055::Read8(_5GHUB_bno055_reg_t reg) {
  uint8_t buffer[1] = {reg};
  i2c_dev->write_then_read(buffer, 1, buffer, 1);
  return (byte)buffer[0];
}

bool _5GHUB_BNO055::ReadLen(_5GHUB_bno055_reg_t reg, byte *buffer,
                              uint8_t len) {
  uint8_t reg_buf[1] = {(uint8_t)reg};
  return i2c_dev->write_then_read(reg_buf, 1, buffer, len);
}
