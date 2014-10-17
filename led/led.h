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
* @file led/led.h
*
* LED definitions
*/
#ifndef LED_H
#define LED_H

#include "mbed.h"

#define LED_TOGGLE(x) x=!x
#define LED_ON(x) x=1
#define LED_OFF(x) x=0

// LEDs
extern DigitalOut led_sys;
extern DigitalOut led_cantx;
extern DigitalOut led_canrx;
extern DigitalOut led_telemetry;

// UTILS
#define RunOnceEvery(_prescaler, _code) {       \
    static uint16_t prescaler = 0;              \
    prescaler++;                                \
    if (prescaler >= _prescaler) {              \
      prescaler = 0;                            \
      _code;                                    \
    }                                           \
}

#endif /* LED_H */
