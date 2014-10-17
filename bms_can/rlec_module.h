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
* @file can_bms/rlec_module.h
*
* RLEC defines
*/
#ifndef RLEC_CAN_H
#define RLEC_CAN_H

#define RLEC_CELLS 12

enum RLECStatus {
  Disabled,
  Active
};

struct RLECModule {
    uint16_t cell_voltage[RLEC_CELLS];
    uint16_t max_cell_volt;
    uint16_t min_cell_volt;
    int8_t rlec_temp;
    uint16_t balance_resistors; // balance regs byte for now
    uint8_t faults; // faults byte
    uint16_t rlec_volt;
    int8_t cell_temp[RLEC_CELLS];
    int8_t min_cell_temp;
    int8_t max_cell_temp;
    uint8_t sw_id;
    uint8_t charge_status;
    enum RLECStatus status;
};



#endif /* RLEC_CAN_H */
