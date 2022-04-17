/*

  Copyright 2019, 5G HUB

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
  following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.

*/

#ifndef __BOARD_H_
#define __BOARD_H_

// LTE & IoT modems
#include "5G-NB-IoT_Common.h"
#include "5G-NB-IoT_ATCommand.h"
#include "5G-NB-IoT_FILE.h"
#include "5G-NB-IoT_GNSS.h"
#include "5G-NB-IoT_HTTP.h"
#include "5G-NB-IoT_MQTT.h"
#include "5G-NB-IoT_Serial.h"
#include "5G-NB-IoT_SSL.h"
#include "5G-NB-IoT_TCPIP.h"
#include "AWSIOT.hpp"

#include "5GHUB_Sensor.h"

#include <utility/imumaths.h>

// BME680 sensor
#include "5GHUB_BME680.h"

// TSL25911 sensor
#include "5GHUB_TSL25911.h"

// BNO055 sensor
#include "5GHUB_BNO055.h"


#include "pindef.h"

#endif
