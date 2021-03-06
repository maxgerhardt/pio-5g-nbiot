/*
 * A library for 5G_NB_IoT Development board
 * This file is about the BG96 Common function
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
#ifndef _5G_NB_IoT_H_
#define _5G_NB_IoT_H_

#include "5G-NB-IoT_Serial.h"

#define POWKEY_PIN  4
#define RESET_PIN   3

typedef enum functionality{
    MINIMUM_FUNCTIONALITY = 0, 
    FULL_FUNCTIONALITY = 1,
    DISABLE_RF = 4,
}Functionality_t;

typedef enum cmd_status{
    READ_MODE = 0,
    WRITE_MODE = 1,
}Cmd_Status_t;

typedef enum net_status{
    NOT_REGISTERED = 0,
    REGISTERED = 1,
    SEARCHING = 2,
    REGISTRATION_DENIED = 3,
    UNKNOWN = 4,
    REGISTERED_ROAMING = 5,
}Net_Status_t;

typedef enum net_type{
    GSM = 0,
    LTE_CAT_M1 = 8,
    LTE_CAT_NB1 = 9,
}Net_Type_t;


class _5G_NB_IoT_Common : public _5G_NB_IoT_Serial
{
  public:
    _5G_NB_IoT_Common();

    ~_5G_NB_IoT_Common();

    _5G_NB_IoT_Common(Stream &atserial, Stream &dserial);

    bool TurnOnModule();
	
    bool InitModule();

    bool ResetModule();

    bool SetDevCommandEcho(bool echo);

    bool GetDevInformation(char *inf);

    bool GetDevVersion(char *ver);

    bool GetDevIMEI(char *imei);

    Cmd_Response_t SetDevFunctionality(Functionality_t mode);

    bool DevLocalRate(unsigned long &rate, Cmd_Status_t status);

    bool GetDevSimIMSI(char *imsi);

    bool DevSimPIN(char *pin, Cmd_Status_t status);

    bool GetDevSimICCID(char *iccid);

    Net_Status_t DevNetRegistrationStatus();

    bool GetDevNetSignalQuality(unsigned int &rssi);

    Cmd_Response_t ScanOperatorNetwork(char *net);

    Cmd_Response_t DevOperatorNetwork(unsigned int &mode, unsigned int &format, char *oper, Net_Type_t &act, Cmd_Status_t status);

    bool GetDevNetworkInformation(char *type, char *oper, char *band, char *channel);

    bool DevNetPacketCounter(unsigned long &send_bytes, unsigned long &recv_bytes, bool clean);

    bool DevPowerDown();

    bool DevClock(char *d_clock, Cmd_Status_t status);
#if 0
    bool ConfigPin(int pin, int dir, int pull, int drv);

    bool PinWrite(int pin, int state);
#endif
    //0 Automatic, 1 GSM only, 3 LTE only
    bool ScanmodeConfig(int mode);

  private:

};

#endif