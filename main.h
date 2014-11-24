/*
* Copyright (C) 2014 Viking Motorsport Team (VMS)
*
* This file is part of VMS Battery Management System.
*
* VMS Battery Management System is free software; you can redistribute 
* it and/or modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2,
* or (at your option) any later version.
*
* VMS Battery Management System is distributed in the hope that
* it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*/

/**
* @file main.hpp
*
* BMS main loop.
*/
#ifndef MAIN_H
#define MAIN_H

#include "mbed.h"


// FAULT DEFINES
#define RLEC_CELL_1_AD_FAULT 0x1 // critical
#define RLEC_CELL_TEMP_AD_FAULT 0x2 // warning
#define RLEC_RLEC_TEMP_AD_FAULT 0x4 // warning
#define RLEC_CELL_VOLTAGE_CONNECTION_FAULT 0x8 //critical
#define RLEC_CELL_VOLTAGE_AD_FAULT 0x10 // critical
#define RLEC_MODULE_VOLTAGE_AD_FAULT 0x20 // critical
#define RLEC_ZERO_CAPACITOR_VOLTAGE_FAULT 0x40 // nothing
#define RLEC_CELL_1_VOLTAGE_FAULT 0x80 // critical

#define T_MIN 100 // s*10

#define MAX_CELL_VOLT (4.2/RLEC_CAN_VOLTAGE_MULT)
#define MIN_CELL_VOLT (2.5/RLEC_CAN_VOLTAGE_MULT)

#define MAX_CELL_TEMP 50

#define BAT_LOW (3/RLEC_CAN_VOLTAGE_MULT)
#define BAT_VERY_LOW (2.7/RLEC_CAN_VOLTAGE_MULT)
#define BAT_CRITICAL MIN_CELL_VOLT

#define BAT_BALANCE_CUTOFF (4.13/RLEC_CAN_VOLTAGE_MULT)
#define BAT_CHARGE_CUTOFF (4.15/RLEC_CAN_VOLTAGE_MULT)
#define BAT_DELTA_BALANCE 8

extern DigitalOut llim;
extern DigitalOut hlim;
extern DigitalOut fw_enable;

extern float s1;
extern float s2;
extern float out;


/*
extern AnalogOut acc_out;
extern AnalogIn throttle1;
extern AnalogIn throttle2;
*/

inline void main_init( void );
inline void main_event( void );
inline void handle_periodic_tasks( void );

inline void failsafe_periodic( void );
inline void failsafe_shutdown( void );
inline void failsafe_warning( void );
inline void datalog_periodic(void);

inline void charger_shutdown( void );
inline void batlow_warning( void );
inline void batverylow_warning( void );
inline void batcritical_warning( void );

void telemetry_tid(void);
void downlink_tid(void);
void failsafe_tid(void);
void can_tid(void);
void heartbeat_tid(void);
void datalog_tid(void);

bool throttle_plausibility(float sig1, float sig2, float *out);
inline float f_abs(float val);

#endif /* MAIN_H */
