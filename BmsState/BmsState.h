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
* @file BmsState/BmsState.h
*
* Class definition for the BMS state
*/
#ifndef BMS_STATE_H
#define BMS_STATE_H

#include "mbed.h"
/*
class Date
{
private:
    int m_nMonth;
    int m_nDay;
    int m_nYear;
 
    Date() { } // private default constructor
 
public:
    Date(int nMonth, int nDay, int nYear);
 
    void SetDate(int nMonth, int nDay, int nYear);
 
    int GetMonth() { return m_nMonth; }
    int GetDay()  { return m_nDay; }
    int GetYear() { return m_nYear; }
};
*/
enum BmsStatus {
  BMS_Uninit,
  BMS_Running,
  BMS_Fault
};

enum VSMstate {
  VSM_start,
  VSM_precharge_init,
  VSM_precharge_active,
  VSM_precharge_complete,
  VSM_wait,
  VSM_ready,
  VSM_running,
  VSM_fault,
  VSM_shutdown,
  VSM_recycle
};

enum InverterState {
    Inv_power_on,       
    Inv_stop,
    Inv_open_loop,
    Inv_closed_loop,
    Inv_wait,
    Inv_internal_1,
    Inv_internal_2,
    Inv_internal_3,
    Inv_idle_run,
    Inv_idle_stop,
    Inv_internal_4,
    Inv_internal_5,
    Inv_internal_6
};

enum InvRunMode {
    InvRun_Torque_Mode,
    InvRun_Speed_Mode
};

enum InvCmdMode {
    InvCmd_CAN,
    InvCmd_VSM,
};


class BmsState
{
    public:
        BmsStatus status;
        
        // Time
        uint32_t timer; // u
        uint16_t up_time; // u
        
        // Temps
        int16_t phase_temp[3];//3x i
        int16_t gate_temp;// i
        int16_t board_temp;// i
        int16_t rtd_temp[5];// 5x i
        int16_t motor_temp;// i
        
        // Torque
        int16_t torque_shud;// i
        int16_t torque_cmd;// i
        int16_t torque_fb;// i
        
        // Analog inputs
        int16_t analog_in[4];//4x i
        
        // Digital inputs
        uint8_t digital_in[6];//6x u
        
        // Motor info
        int16_t motor_angle; //i
        int16_t motor_speed;// i
        int16_t inv_freq;// i
        int16_t resolver_angle;// i
        
        // Current
        int16_t phase_current[3]; // Phase A, B, C 3xi
        int16_t dc_current;// i
        
        // Voltage
        int16_t dc_voltage;// i
        int16_t output_volt;// i
        int16_t p_ab_volt;// i
        int16_t p_bc_volt;// i
        
        // Flux
        int16_t flux_cmd;//i
        int16_t flux_fb;// i
        int16_t id_fb;// i
        int16_t iq_fb;// i
        int16_t id_cmd;// i
        int16_t iq_cmd;// i
        
        // Internal Voltages
        int16_t ref_1_5;// i
        int16_t ref_2_5;// i
        int16_t ref_5_0;// i
        int16_t sys_12v;// i
        
        // Internal State
        enum VSMstate vsm_state;// u
        enum InverterState inv_state;// u
        uint8_t relay_state;// u
        enum InvRunMode inv_mode;// u
        enum InvCmdMode inv_cmd;// u
        uint8_t inv_enable;// u
        uint8_t direction;// u
        uint8_t faults[8];// u
        
        // Various
        int16_t modulation_index;// i
        int16_t flux_reg_out;// i
        
        // Firmware data
        uint16_t eeprom_version;
        uint16_t sw_version;

        // Cell data
        int8_t max_cell_temp;
        int8_t min_cell_temp;
        uint16_t max_cell_volt;
        uint16_t min_cell_volt;

        // Default constructor        
        BmsState(void);
};

extern BmsState bms;

void state_init(void);

#endif /* BMS_STATE_H */
