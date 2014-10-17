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
* @file can_bms/mlec_bms.h
*
* CAN interface for RLECs
*/
#ifndef MLEC_CAN_H
#define MLEC_CAN_H

#include "mbed.h"
#include "led.h"
#include "rlec_module.h"

#define RLEC_CAN_VOLTAGE_MULT 0.00244
#define RLEC_CAN_MODULE_MULT 0.0122
#define RLEC_CAN_FREQUENCY 500000
#define NUM_RLECS 16

enum MLECStatus {
  MLECUninit,
  MLECInit,
  MLECRunning
};

struct MLECCan {
    // Rx message
    CANMessage rx_msg;
    
    // RLEC request msg
    CANMessage tMsg0;
    CANMessage tMsg1;
    CANMessage tMsg2;
    CANMessage tMsg3;
    
    // Broadcast msgs
    CANMessage bdc0;
    CANMessage bdc1;
    CANMessage bdc2;
    CANMessage bdc3;
    CANMessage bdc4;
    CANMessage bdc5;

    // array with active RLECS
    uint8_t rlecs[NUM_RLECS];
    struct RLECModule rlecsX[NUM_RLECS];
    
    // Index of current RLECs probe
    uint8_t rlec_idx;
    
    // Tx Message offset
    uint8_t rlec_txoffset;
    
    // Status
    enum MLECStatus status;
    
    // Variables for RLEC scan
    uint8_t request_sent; 
};

extern struct MLECCan mlec;

void can_init_rlecs(void);
void can_periodic_rlecs(void);
void can_event_rlecs(void);

void mlec_init_broadcast(void);
void mlec_init_msgs(void);
void mlec_update_msg_ids(uint16_t);
uint8_t mlec_broadcast(void);
uint8_t mlec_send_request(void);

void rlec_parse_msg(void);
void rlec_parse_scan(int);

void mlec_charger_periodic(RLECModule* module, CANMessage* msg6);

extern DigitalOut hlim;

#endif /* MLEC_CAN_H */

