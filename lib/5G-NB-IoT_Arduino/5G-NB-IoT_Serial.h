/*
 * A library for 5G-NB-IoT Development board
 * This file is about the serial defined
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

#ifndef __5G_NB_IoT_SERIAL_H_
#define __5G_NB_IoT_SERIAL_H_

#include "string.h"
#include "stdlib.h"
#include "Arduino.h"
#include "5G-NB-IoT_ATCommand.h"

#define RX_BUFFER_LENGTH  1024
#define UART_DEBUG

const unsigned long Band_list[] = {
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    460800,
    921600
};

typedef enum cmd_response{
    UNKNOWN_RESPONSE  = -2,
    TIMEOUT_RESPONSE  = -1,
    FIAL_RESPONSE  =  0,
    SUCCESS_RESPONSE  = 1,
}Cmd_Response_t;

class _5G_NB_IoT_Serial
{
  public:
    _5G_NB_IoT_Serial();

    ~_5G_NB_IoT_Serial();

    _5G_NB_IoT_Serial(Stream &atserial,Stream &dserial);

    void AT_bypass();

    template <class T>
    bool AutoATSerialBand(T &_atserial);

    bool sendDataAndCheck(const char *data_buf, const char *ok_str, const char *err_str, unsigned int timeout);

    bool sendATcommand(const char *command);

    unsigned int readResponseByteToBuffer();

    unsigned int readResponseToBuffer(unsigned int timeout);

    Cmd_Response_t readResponseAndSearch(const char *test_str, unsigned int timeout);

    Cmd_Response_t readResponseAndSearchChr(const char test_chr, unsigned int timeout);

    Cmd_Response_t readResponseAndSearch(const char *test_str, const char *e_test_str, unsigned int timeout);

    Cmd_Response_t sendAndSearch(const char *command, const char *test_str, unsigned int timeout);

    Cmd_Response_t sendAndSearchChr(const char *command, const char test_chr, unsigned int timeout);

    Cmd_Response_t sendAndSearch(const char *command, const char *test_str, const char *e_test_str, unsigned int timeout);

    bool returnErrorCode(int &s_err_code);

    void cleanBuffer();

    int serialAvailable();

    char *searchStrBuffer(const char *test_str);

    char *searchChrBuffer(const char test_chr);

    unsigned int bufferHead;
    char rxBuffer[RX_BUFFER_LENGTH];
    int errorCode = -1;

  private:
    Stream& _atserial;
    Stream& _dserial;


};

#endif