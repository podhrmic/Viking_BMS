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
* @file can_bms/rms_can.c
*
* CAN interface for RMS controller
*/
#include "rms_can.h"

//CAN
CAN canbus_rms(p30, p29);

// Message
CANMessage rms_rx_msg;


/**
 * Configure CAN
 */
void can_init_rms(void) {
    canbus_rms.frequency(RMS_CAN_FREQUENCY);
    bms.faults[0] = 0;
    bms.faults[1] = 0;
    bms.faults[2] = 0;
    bms.faults[3] = 0;
    bms.faults[4] = 0;
    bms.faults[5] = 0;
    bms.faults[6] = 0;
    bms.faults[7] = 0;
}

/**
 * Periodic CAN function
 *
 * Empty for now
 */
void can_periodic_rms(void) {
    // TBD    
}

/**
 * Check and parse messages
 */
void can_event_rms(void) {
    if(canbus_rms.read(rms_rx_msg)) {
        // Parse msg
        rms_parse_msg();
    }
}

/**
 * Parse message during normal operation
 */
inline void rms_parse_msg(void) {
    switch(rms_rx_msg.id){
        case 0xA0: // Temperatures #1
            bms.phase_temp[0] = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.phase_temp[1] = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.phase_temp[2] = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.gate_temp = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA1: // Temperatures #2
            bms.board_temp = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.rtd_temp[0] = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.rtd_temp[1] = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.rtd_temp[2] = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA2: // Temperatures #3
            bms.rtd_temp[3] = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.rtd_temp[4] = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.motor_temp = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.torque_shud = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA3: // Analog Input Voltages
            bms.analog_in[0] = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.analog_in[1] = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.analog_in[2] = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.analog_in[3] = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA4: // Digital Input Status
            bms.digital_in[0] = rms_rx_msg.data[0];
            bms.digital_in[1] = rms_rx_msg.data[1];
            bms.digital_in[2] = rms_rx_msg.data[2];
            bms.digital_in[3] = rms_rx_msg.data[3];
            bms.digital_in[4] = rms_rx_msg.data[4];
            bms.digital_in[5] = rms_rx_msg.data[5];
            break;
        case 0xA5: // Motor Position Information
            bms.motor_angle = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.motor_speed = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.inv_freq = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.resolver_angle = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA6: // Current Information
            bms.phase_current[0] = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.phase_current[1] = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.phase_current[2] = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.dc_current = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA7: // Voltage Information
            bms.dc_voltage = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.output_volt = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.p_ab_volt = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.p_bc_volt = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA8: // Flux Information
            bms.flux_cmd = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.flux_fb = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.id_fb = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.iq_fb = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xA9: // Internal Voltages
            bms.ref_1_5 = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.ref_2_5 = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.ref_5_0 = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.sys_12v = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xAA: // Internal States
            bms.vsm_state = (VSMstate)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.inv_state = (InverterState)rms_rx_msg.data[2];
            bms.relay_state = rms_rx_msg.data[3];
            bms.inv_mode = (InvRunMode)rms_rx_msg.data[4];
            bms.inv_cmd = (InvCmdMode)rms_rx_msg.data[5];
            bms.inv_enable = rms_rx_msg.data[6];
            bms.direction = rms_rx_msg.data[7];
            break;
        case 0xAB: // Fault Codes
            bms.faults[0] = rms_rx_msg.data[0];
            bms.faults[1] = rms_rx_msg.data[1];
            bms.faults[2] = rms_rx_msg.data[2];
            bms.faults[3] = rms_rx_msg.data[3];
            bms.faults[4] = rms_rx_msg.data[4];
            bms.faults[5] = rms_rx_msg.data[5];
            bms.faults[6] = rms_rx_msg.data[6];
            bms.faults[7] = rms_rx_msg.data[7];
            break;
        case 0xAC: // Torque & Timer Info
            bms.torque_cmd = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.torque_fb = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.timer = (uint32_t)(rms_rx_msg.data[7] << 24 | rms_rx_msg.data[6] << 16 | rms_rx_msg.data[5] << 8  | rms_rx_msg.data[4] );
            break;
        case 0xAD: // Modulation Index & Flux Weakening Output Information
            bms.modulation_index = (int16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.flux_reg_out = (int16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            bms.id_cmd = (int16_t)(rms_rx_msg.data[5] << 8 | rms_rx_msg.data[4]);
            bms.iq_cmd = (int16_t)(rms_rx_msg.data[7] << 8 | rms_rx_msg.data[6]);
            break;
        case 0xAE: // Firmware Information
            bms.eeprom_version = (uint16_t)(rms_rx_msg.data[1] << 8 | rms_rx_msg.data[0]);
            bms.sw_version = (uint16_t)(rms_rx_msg.data[3] << 8 | rms_rx_msg.data[2]);
            break;
        case 0xAF: // Diagnostic Data
            // Ignore
            break;
        default:
            break;
    }
}

