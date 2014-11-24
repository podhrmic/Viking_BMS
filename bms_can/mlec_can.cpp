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
#include "mlec_can.h"
#include "downlink.h"
#include "main.h"

#define STANDARD_CHARGE_MODE 0

//CAN
CAN canbus_mlec(p9, p10);

// MLEC struct
struct MLECCan mlec;

/**
 * Configure CAN
 */
void can_init_rlecs(void)
{
    // MLEC setup
    mlec.status = MLECUninit;
    memset(mlec.rlecs, 0, sizeof(mlec.rlecs));
    mlec.rlec_idx = 0;
    mlec.rlec_txoffset = 0x40;
    mlec_init_broadcast();
    mlec_init_msgs();
    mlec_update_msg_ids(mlec.rlec_txoffset);
    mlec.request_sent = 0;
    
    // CANbus setup
    canbus_mlec.frequency(RLEC_CAN_FREQUENCY);
    
    LED_ON(led_cantx);
}

/**
 * Periodic CAN function
 *
 * Scan and ask for data
 */
 void can_periodic_rlecs(void){
    LED_TOGGLE(led_cantx);
    switch(mlec.status){
        case MLECUninit:
            if (mlec_broadcast() && mlec_send_request()) {
                mlec.request_sent = 1;
                mlec.status = MLECInit;
            }
            break;
        case MLECInit:
            debuglink.printf("Scanned RLEC%i, ", mlec.rlec_idx);
            debuglink.printf("Found: %i\r\n", mlec.rlecsX[mlec.rlec_idx].status);

            if (mlec.rlec_idx < (NUM_RLECS-1)) {
                // Scanning
                mlec.rlec_idx++;
                mlec.rlec_txoffset = mlec.rlec_txoffset+0x2;
                mlec_update_msg_ids(mlec.rlec_txoffset);
                mlec_send_request();
                mlec.request_sent = 1;
            }
            else {
                mlec.rlec_idx = 0;
                mlec.rlec_txoffset = 0x40;
                mlec.status = MLECRunning; // Scan complete
            }
            break; 
        case MLECRunning:
            // Good for now, although it looses time if waiting for rlecs that arent 
            // present, maybe some resolution to move up idx to actual values of RLECS 
            // (only 1 in the array)
            if (mlec.rlecsX[mlec.rlec_idx].status == Active) {
                // RLEC idx present
                mlec_update_msg_ids(mlec.rlec_txoffset+0x2*mlec.rlec_idx);
                
                // Modify messages
                mlec_charger_periodic(&mlec.rlecsX[mlec.rlec_idx], &mlec.tMsg0);
                
                mlec_send_request();
            }
            //set messages to default
            mlec_init_msgs();
            mlec.rlec_idx++;
            mlec.rlec_idx = mlec.rlec_idx % NUM_RLECS;
            break;
        default:
            break;    
    }
}

/**
 * Check if we received messages
 */
void can_event_rlecs(void) {
    if(canbus_mlec.read(mlec.rx_msg)) {
        RunOnceEvery(10,LED_TOGGLE(led_canrx));
        if (mlec.status == MLECRunning) {
           // Normal operation
           rlec_parse_msg();
        }
        else {
           // Scan operation
           rlec_parse_scan(mlec.rx_msg.id); 
        }    
    }
}

/**
 * Parse message during normal operation
 */
void rlec_parse_msg(void) {
    static uint8_t msg_offset, msg_id, idx;
    msg_offset = (uint8_t)(mlec.rx_msg.id >> 4);
    msg_id = (uint8_t)(mlec.rx_msg.id & 0xF);
    idx = msg_offset/2;
    
    if (idx >= NUM_RLECS) idx = 0;
    switch(msg_id) {
        case 0x1: // Cell Voltage 1-4
            mlec.rlecsX[idx].cell_voltage[0] = (uint16_t)(mlec.rx_msg.data[0] << 8 | mlec.rx_msg.data[1]);
            mlec.rlecsX[idx].cell_voltage[1] = (uint16_t)(mlec.rx_msg.data[2] << 8 | mlec.rx_msg.data[3]);
            mlec.rlecsX[idx].cell_voltage[2] = (uint16_t)(mlec.rx_msg.data[4] << 8 | mlec.rx_msg.data[5]);
            mlec.rlecsX[idx].cell_voltage[3] = (uint16_t)(mlec.rx_msg.data[6] << 8 | mlec.rx_msg.data[7]);
            break;
        case 0x2: // Cell Voltage 5-8
            mlec.rlecsX[idx].cell_voltage[4] = (uint16_t)(mlec.rx_msg.data[0] << 8 | mlec.rx_msg.data[1]);
            mlec.rlecsX[idx].cell_voltage[5] = (uint16_t)(mlec.rx_msg.data[2] << 8 | mlec.rx_msg.data[3]);
            mlec.rlecsX[idx].cell_voltage[6] = (uint16_t)(mlec.rx_msg.data[4] << 8 | mlec.rx_msg.data[5]);
            mlec.rlecsX[idx].cell_voltage[7] = (uint16_t)(mlec.rx_msg.data[6] << 8 | mlec.rx_msg.data[7]);
            break;
        case 0x3: // Cell Voltage 9-12
            mlec.rlecsX[idx].cell_voltage[8] = (uint16_t)(mlec.rx_msg.data[0] << 8 | mlec.rx_msg.data[1]);
            mlec.rlecsX[idx].cell_voltage[9] = (uint16_t)(mlec.rx_msg.data[2] << 8 | mlec.rx_msg.data[3]);
            mlec.rlecsX[idx].cell_voltage[10] = (uint16_t)(mlec.rx_msg.data[4] << 8 | mlec.rx_msg.data[5]);
            mlec.rlecsX[idx].cell_voltage[11] = (uint16_t)(mlec.rx_msg.data[6] << 8 | mlec.rx_msg.data[7]);
            break;
        case 0x4: // Max/Min cell voltage, RLEC temp, balance, faults
            mlec.rlecsX[idx].max_cell_volt = (uint16_t)(mlec.rx_msg.data[0] << 8 | mlec.rx_msg.data[1]);
            mlec.rlecsX[idx].min_cell_volt = (uint16_t)(mlec.rx_msg.data[2] << 8 | mlec.rx_msg.data[3]);
            mlec.rlecsX[idx].rlec_temp = mlec.rx_msg.data[4];
            mlec.rlecsX[idx].balance_resistors = (uint16_t)( (mlec.rx_msg.data[5] << 8 | mlec.rx_msg.data[6]) & 0xFFF);
            //mlec.rlecsX[idx].balance_resistors = (uint16_t)( (mlec.rx_msg.data[5] << 8 | mlec.rx_msg.data[6]));
            mlec.rlecsX[idx].faults = mlec.rx_msg.data[7];
            break;
        case 0x5: // Unfiltered Cell Voltage 1-4
            // Not used
            break;
        case 0x6: // Unfiltered Cell Voltage 5-8
            // Not used
            break;
        case 0x7: // Unfiltered Cell Voltage 9-12
            // Not used
            break;
        case 0x8: // Raw Cell Voltage 1-4
            // Not used
            break;
        case 0x9: // Raw Cell Voltage 5-8
            // Not used
            break;
        case 0xA: // Raw Cell Voltage 9-12
            // Not used
            break;
        case 0xB: // RLEC module voltage, 
            mlec.rlecsX[idx].rlec_volt = (uint16_t)(mlec.rx_msg.data[6] << 8 | mlec.rx_msg.data[7]);
            break;
        case 0xC: // Cell temp 1-8
            mlec.rlecsX[idx].cell_temp[0] = mlec.rx_msg.data[0];
            mlec.rlecsX[idx].cell_temp[1] = mlec.rx_msg.data[1];
            mlec.rlecsX[idx].cell_temp[2] = mlec.rx_msg.data[2];
            mlec.rlecsX[idx].cell_temp[3] = mlec.rx_msg.data[3];
            mlec.rlecsX[idx].cell_temp[4] = mlec.rx_msg.data[4];
            mlec.rlecsX[idx].cell_temp[5] = mlec.rx_msg.data[5];
            mlec.rlecsX[idx].cell_temp[6] = mlec.rx_msg.data[6];
            mlec.rlecsX[idx].cell_temp[7] = mlec.rx_msg.data[7];
            break;
        case 0xD: // Cell temp 9-12, min/max cell temp, SW id
            mlec.rlecsX[idx].cell_temp[8] = mlec.rx_msg.data[0];
            mlec.rlecsX[idx].cell_temp[9] = mlec.rx_msg.data[1];
            mlec.rlecsX[idx].cell_temp[10] = mlec.rx_msg.data[2];
            mlec.rlecsX[idx].cell_temp[11] = mlec.rx_msg.data[3];
            mlec.rlecsX[idx].max_cell_temp = mlec.rx_msg.data[4];
            mlec.rlecsX[idx].min_cell_temp = mlec.rx_msg.data[5];
            mlec.rlecsX[idx].sw_id = mlec.rx_msg.data[7];
            break;            
        default:
            break;
        }   
}

/**
 * Parse message during scan
 */
void rlec_parse_scan(int msg_id) {
    if ((msg_id >> 4) == (mlec.rlec_txoffset-0x40)) {//offset between Tx and Rx msgs for a particular RLEC
        // expected value arrived, notify periodic
        mlec.request_sent = 0;
        
        // write in buffer
        mlec.rlecsX[mlec.rlec_idx].status = Active;
    }
}    

/* 
 * Charger function
 *
 */
void mlec_charger_periodic(RLECModule* module, CANMessage* msg6 ) {
 uint32_t balance_resistors = 0;
 if (STANDARD_CHARGE_MODE){
    //debuglink.printf("STANDARD CHARGE MODE\r\n");
    // standard charge mode
    if ((module->max_cell_volt >= BAT_CHARGE_CUTOFF) || (module->charge_status == 1)) {
        // switch off charger
        if (hlim == 1) {
            hlim = 0;
            module->charge_status = 1;    
        }
        
        
        // update status
        //TBD
        
        for (uint8_t j = 0; j<RLEC_CELLS;j++) {
        // if any cell is more than delta from the lowest cell
            if ((module->cell_voltage[j] - module->min_cell_volt) >= BAT_DELTA_BALANCE) {
                balance_resistors = balance_resistors + (0x1 << j);
            }
            //debuglink.printf("delta: %d\r\n", (module->cell_voltage[j] - module->min_cell_volt));
        }
        
        if (balance_resistors == 0) {
            module->charge_status = 0;
        }
        //debuglink.printf("delta: %d\r\n", BAT_DELTA_BALANCE);
        
        //debuglink.printf("BALANCING\r\n");
    }
 }
 else {
//debuglink.printf("BALANCE CHARGE MODE\r\n");
 // COOL BALANCE MODE    
    // we dont want charging and balancing at the same time
    //hlim = 1;
    for (uint8_t j = 0; j<RLEC_CELLS;j++) {
        if (module->cell_voltage[j] > BAT_BALANCE_CUTOFF) {
            balance_resistors = balance_resistors + (0x1 << j);
        }
    }
 }
//debuglink.printf("balance resistors: %X\r\n", balance_resistors);
// Configure message to given RLEC
msg6->data[3] = (uint8_t)(balance_resistors >> 8);
msg6->data[4] = (uint8_t)(balance_resistors & 0xFF);
}

/**
 * mlec_broadcast
 *
 * Send broadcast messages
 */
uint8_t mlec_broadcast(void){
    static uint8_t counter = 0;
    counter = counter + canbus_mlec.write(mlec.bdc0);
    wait(0.007); // busy wait (NOP)
    counter = counter + canbus_mlec.write(mlec.bdc1);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc2);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc3);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc4);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc5);
    wait(0.007);
    if (counter != 6){ // not all messages were OK
        return 0;
    }
    else return 1; // broadcast sucessful
}

/**
 * mlec_send_request
 *
 * Send data request for a specific RLEC. 
 * Call this function after setting the right msg ids
 */
uint8_t mlec_send_request(void){
    static uint8_t counter = 0;
    counter = counter + canbus_mlec.write(mlec.tMsg0);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg1);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg2);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg3);
    if (counter != 4){ // not all messages were OK
        return 0;
    }
    else return 1; // send sucessful
}

/**
 * mlec_update_msg_id
 *
 * Updates msg ids
 */
void mlec_update_msg_ids(uint16_t offset){
    offset = offset << 4;
    mlec.tMsg0.id = offset | 0x6;
    mlec.tMsg1.id = offset | 0xA;
    mlec.tMsg2.id = offset | 0xB;
    mlec.tMsg3.id = offset | 0xC;
}

/**
 * mlec_init_msgs
 *
 * Fills in data request messages
 */
void mlec_init_msgs(void){
    // from the TH!NK datasheet
    // MSG 0, id = 0xXX6
    mlec.tMsg0.len = 8;
    mlec.tMsg0.data[0] = 0x0F;
    mlec.tMsg0.data[1] = 0x0F;
    mlec.tMsg0.data[2] = 0xFF;
    mlec.tMsg0.data[3] = 0x00;
    mlec.tMsg0.data[4] = 0x00;
    mlec.tMsg0.data[5] = 0x00;
    mlec.tMsg0.data[6] = 0x00;
    mlec.tMsg0.data[7] = 0x00;
    mlec.tMsg0.type = CANData;
    mlec.tMsg0.format  = CANStandard;

    // MSG 1, id = 0xXXA
    mlec.tMsg1.len = 8;
    mlec.tMsg1.data[0] = 0x00;
    mlec.tMsg1.data[1] = 0x00;
    mlec.tMsg1.data[2] = 0x00;
    mlec.tMsg1.data[3] = 0x00;
    mlec.tMsg1.data[4] = 0x00;
    mlec.tMsg1.data[5] = 0x00;
    mlec.tMsg1.data[6] = 0x00;
    mlec.tMsg1.data[7] = 0x00;
    mlec.tMsg1.type = CANData;
    mlec.tMsg1.format  = CANStandard;

    // MSG 2, id = 0xXXB
    mlec.tMsg2.len = 8;
    mlec.tMsg2.data[0] = 0x00;
    mlec.tMsg2.data[1] = 0x00;
    mlec.tMsg2.data[2] = 0x00;
    mlec.tMsg2.data[3] = 0x00;
    mlec.tMsg2.data[4] = 0x00;
    mlec.tMsg2.data[5] = 0x00;
    mlec.tMsg2.data[6] = 0x00;
    mlec.tMsg2.data[7] = 0x00;
    mlec.tMsg2.type = CANData;
    mlec.tMsg2.format  = CANStandard;

    // MSG 3, id = 0xXXC
    mlec.tMsg3.len = 8;
    mlec.tMsg3.data[0] = 0x00;
    mlec.tMsg3.data[1] = 0x00;
    mlec.tMsg3.data[2] = 0x00;
    mlec.tMsg3.data[3] = 0x00;
    mlec.tMsg3.data[4] = 0x00;
    mlec.tMsg3.data[5] = 0x00;
    mlec.tMsg3.data[6] = 0x0C;
    mlec.tMsg3.data[7] = 0x0C;
    mlec.tMsg3.type = CANData;
    mlec.tMsg3.format  = CANStandard;
   
}


/**
 * mlec_init_broadcast
 *
 * Fills in universal broadcast messages
 */
void mlec_init_broadcast(void){
    // from the TH!NK datasheet
    mlec.bdc0.id = 0x7e1;
    mlec.bdc0.len = 8;
    mlec.bdc0.data[0] = 0x01;
    mlec.bdc0.data[1] = 0x0C;
    mlec.bdc0.data[2] = 0x0C;
    mlec.bdc0.data[3] = 0x01;
    mlec.bdc0.data[4] = 0x01;
    mlec.bdc0.data[5] = 0x00;
    mlec.bdc0.data[6] = 0x05;
    mlec.bdc0.data[7] = 0xBD;
    mlec.bdc0.type = CANData;
    mlec.bdc0.format = CANStandard;

    mlec.bdc1.id = 0x7e2;
    mlec.bdc1.len = 8;
    mlec.bdc1.data[0] = 0x00;
    mlec.bdc1.data[1] = 0x0A;
    mlec.bdc1.data[2] = 0x00;
    mlec.bdc1.data[3] = 0x0A;
    mlec.bdc1.data[4] = 0x00;
    mlec.bdc1.data[5] = 0x00;
    mlec.bdc1.data[6] = 0x01;
    mlec.bdc1.data[7] = 0x00;
    mlec.bdc1.type = CANData;
    mlec.bdc1.format = CANStandard;

    mlec.bdc2.id = 0x7e3;
    mlec.bdc2.len = 8;
    mlec.bdc2.data[0] = 0x06;
    mlec.bdc2.data[1] = 0xB9;
    mlec.bdc2.data[2] = 0x06;
    mlec.bdc2.data[3] = 0xB9;
    mlec.bdc2.data[4] = 0x00;
    mlec.bdc2.data[5] = 0x09;
    mlec.bdc2.data[6] = 0x00;
    mlec.bdc2.data[7] = 0x03;
    mlec.bdc2.type = CANData;
    mlec.bdc2.format = CANStandard;

    mlec.bdc3.id = 0x7e4;
    mlec.bdc3.len = 8;
    mlec.bdc3.data[0] = 0x00;
    mlec.bdc3.data[1] = 0x09;
    mlec.bdc3.data[2] = 0x46;
    mlec.bdc3.data[3] = 0x2D;
    mlec.bdc3.data[4] = 0x0A;
    mlec.bdc3.data[5] = 0x02;
    mlec.bdc3.data[6] = 0x00;
    mlec.bdc3.data[7] = 0x4B;
    mlec.bdc3.type = CANData;
    mlec.bdc3.format = CANStandard;

    mlec.bdc4.id = 0x7e5;
    mlec.bdc4.len = 8;
    mlec.bdc4.data[0] = 0x05;
    mlec.bdc4.data[1] = 0x02;
    mlec.bdc4.data[2] = 0x03;
    mlec.bdc4.data[3] = 0x51;
    mlec.bdc4.data[4] = 0x03;
    mlec.bdc4.data[5] = 0xAE;
    mlec.bdc4.data[6] = 0x05;
    mlec.bdc4.data[7] = 0xCA;
    mlec.bdc4.type = CANData;
    mlec.bdc4.format = CANStandard;

    mlec.bdc5.id = 0x7e6;
    mlec.bdc5.len = 2;
    mlec.bdc5.data[0] = 0x23;
    mlec.bdc5.data[1] = 0x1E;
    mlec.bdc5.type = CANData;
    mlec.bdc5.format = CANStandard;    
}
