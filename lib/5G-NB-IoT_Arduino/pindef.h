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

#ifndef __PINDEF_H_
#define __PINDEF_H_

//ON/OFF the power supply
#define ONOFF 	1

#define RESET_N	3
#define PWRKEY 	4

#define SCL  21
#define SDA  20
#define AREF 42

#define A0   14
#define A1   15
#define A2   16
#define A3   17
#define A4   18
#define A5   19

#define SCK  13
#define MISO 12
#define MOSI 11
#define SS 	 10

#define PA6	 8
#define PA7  9

#define LED1 25
#define LED2 26

#define USER_BUTTON		0

// This section for the mPCIe board
#define PA8		 	PWRKEY
#define PB11	 	24
#define PB22	 	30
#define PB23	 	31

#define RELAY_CTRL	39

#endif
