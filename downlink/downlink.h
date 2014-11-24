/*
* The MIT License (MIT)
*
* Copyright (c) 2014, Michal Podhradsky, Viking Motorsports
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/**
* @file downlink/downlink.h
*
* Telemetry for BMS
*/
#ifndef DOWNLINK_H
#define DOWNLINK_H

#include "mbed.h"
#include "BmsState.h"
#include "led.h"

#include "mlec_can.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdint.h>

#define DOWNLINK_BAUDRATE 9600//115200
#define DEBUGLINK_BAUDRATE 115200
#define TELEMETRY_BAUDRATE 57600

#define DOWNLINK_BUFFER_SIZE 128
#define TELEMETRY_BUFFER_SIZE 128

/*
 * Message headers
 */
#define TELEMETRY_MSG0 0x21
#define TELEMETRY_MSG1 0x3F

/* Data offset */
#define TELEMETRY_SIZE_IDX 2
#define TELEMETRY_DATA_IDX 4

extern Serial debuglink;

void downlink_init(void);
void downlink_event(void);
void downlink_periodic(void);
void telemetry_periodic(void);

void downlink_parse(char);
void downlink_process_cmd(void);

void debuglink_print_rlec(struct RLECModule*);

string toHexString(uint8_t f);
string toHexString(uint16_t f);

extern AnalogOut acc_out;
extern AnalogIn throttle1; //s1
extern AnalogIn throttle2; //s2

extern DigitalIn brake_en;
extern DigitalIn ksi;

int16_t boundNumberShort(int16_t num, int16_t lower, int16_t upper);
uint16_t boundNumberUnsigShort(uint16_t num, uint16_t lower, uint16_t upper);
uint8_t boundNumberUnsigByte(uint8_t num, uint8_t lower, uint8_t upper);

void telemetry_cksum(uint16_t packet_length, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );

//debug
extern bool throttle_ok;
extern float epsilon;
extern bool init;
extern int counter;
extern float fsig1, fsig2,delta;

extern char logname[];
extern FILE *fp;

enum DownlinkState {
  DataMode,
  ATAttempt,
  ATMode
};
#endif /* DOWNLINK_H */
