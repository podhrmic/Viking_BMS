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
* @file main.cpp
*
* BMS main loop.
*/
#include "main.h"
#include "led.h"
#include "BmsState.h"
#include "downlink.h"
#include "mlec_can.h"
#include "rms_can.h"

#ifndef CAN_FREQUENCY
#define CAN_FREQUENCY 10.0
#endif

#ifndef TELEMETRY_FREQUENCY
#define TELEMETRY_FREQUENCY 10.0
#endif

#ifndef DOWNLINK_FREQUENCY
#define DOWNLINK_FREQUENCY 1.0
#endif

#ifndef MODULES_FREQUENCY
#define MODULES_FREQUENCY 1.0
#endif

#ifndef FAILSAFE_FREQUENCY
#define FAILSAFE_FREQUENCY 10.0
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 10.0
#endif

#ifndef DATALOG_FREQUENCY
#define DATALOG_FREQUENCY 1.0
#endif

bool throttle_ok;
float epsilon;
bool init;
int counter;
float fsig1, fsig2, delta;

// Tickers
Ticker heartbeat_tic;
Ticker failsafe_tic;
Ticker telemetry_tic;
Ticker downlink_tic;
Ticker can_tic;
Ticker charger_tic;
Ticker datalog_tic;

// Ticker flags
uint8_t flag_heartbeat;
uint8_t flag_failsafe;
uint8_t flag_telemetry;
uint8_t flag_downlink;
uint8_t flag_can;
uint8_t flag_datalog;

// IO pins
//lov_v - p24
//bms_err - p 25
//crit_vol - p26
DigitalOut bms_err_led(p25);
DigitalOut rtds_enable(p24); // to RTDS
DigitalOut fw_enable(p22);//fw_enable(p26); // to fuse box

DigitalOut llim(p23); // to fuse box - shutdown circuit
DigitalOut hlim(p26);//hlim(p22); // to fuse box

DigitalIn brake_en(p12);//interlock - if High then 0, 1 = no voltage or GND
DigitalIn ksi(p11);

float s1;
float s2;
float out;



/**
 * Heartbeat
 */
void heartbeat()
{
    LED_TOGGLE(led_sys);
    bms.up_time++;
}


/**
 * Main function
 *
 * main_init() initializes peripherals
 * handle_periodic_taks() might be enmpty if we use timers/tickers
 * main_event() handles events (rx message etc)
 *
 */
int main( void ) {
  main_init();

  while(1) {
    handle_periodic_tasks();
    main_event();
  }
}

/**
 * Handle periodic tasks
 *
 * Ticker calls the function within an ISR, so it
 * should not be used for calling bulky or library functions
 * (such as printf). Instead it triggers the flag, which is
 * then handled here.
 */
inline void handle_periodic_tasks(void){
    if (flag_heartbeat) {
        heartbeat();
        flag_heartbeat = 0;
    }
    // start failsafe checks only after T_MIN secs
    // so BMS have time to get CAN responses
    if (flag_failsafe && (bms.up_time > T_MIN)) {
        failsafe_periodic();
        flag_failsafe = 0;
    }
    if (flag_downlink) {
        downlink_periodic();
        flag_downlink = 0;
    }
    if (flag_telemetry) {
        telemetry_periodic();
        flag_telemetry = 0;
    }
    if (flag_can) {
        can_periodic_rlecs();
        can_periodic_rms();
        flag_can = 0;
    }

    if (flag_datalog) {
        datalog_periodic();
        flag_datalog = 0;
    }
}


/**
 * Main_init
 *
 * Initialize I/O, tickers and state machine
 */
inline void main_init(void) {
    // State init
    // in state will be IO pins!
    // so // Initialize output pins too
    state_init(); // state.cpp
        
    //IO init
    bms_err_led = 1; // for demostrative purposes
    llim = 1; // enable AIRS
    hlim = 1; // enable charging
    rtds_enable  = 0; // not play sound (yet)
    fw_enable = 1; // enable fw_switch

    //set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37
    //set_time(1405956440);
    //set_time(1405981640);
    //set_time(1411861442);
    //set_time(1411836242);

    throttle_ok = true;
    epsilon = 0;
    init = false;
    counter = 0;
    fsig1 = 0;
    fsig2 = 0;
    delta = 0;

    acc_out.write(0); // zero throttle at this point
    brake_en.mode(PullUp); // mode doesn't really matter

    // Initialize CAN bus
    can_init_rlecs();
    can_init_rms();
    
    // Start downlinkand logging
    downlink_init(); // downlink.cpp
    
    // Attach tickers
    flag_heartbeat = 0;
    flag_failsafe = 0;
    flag_telemetry = 0;
    flag_downlink = 0;
    flag_can = 0;
    flag_datalog = 0;
    
    heartbeat_tic.attach(&heartbeat_tid, 1.0/HEARTBEAT_FREQUENCY);
    can_tic.attach(&can_tid, 1.0/CAN_FREQUENCY); // can_bms.cpp
    telemetry_tic.attach(&telemetry_tid,1.0/TELEMETRY_FREQUENCY);
    downlink_tic.attach(&downlink_tid,1.0/DOWNLINK_FREQUENCY);
    failsafe_tic.attach(&failsafe_tid,1.0/FAILSAFE_FREQUENCY); 
    datalog_tic.attach(&datalog_tid, 1.0/DATALOG_FREQUENCY);
}

/**
 * Main_event()
 *
 * Events: message/data rx 
 */
inline void main_event(void) {
    // can rx
    can_event_rlecs();
    can_event_rms();
    
    // datalink/serial rx
    downlink_event(); // downlink.cpp
}

/**
 * Datalogging
 */
inline void datalog_periodic(void){
    // File log
    fp = fopen(logname, "a");
    if(fp != NULL) {
   	 //debuglink.printf("Writting, time=%f\r\n",(float)bms.up_time/10);
//   	 printf(fp, "%u, %u,%i, %i, %i,%i, %i, %i, %i, %i, %i, %i,%i,%i, %i, %i,%i, %i, %i, %i,%u, %u, %u, %u, %u, %u,%i, %i, %i, %i,%i, %i, %i,%i,%i, %i, %i, %i,%i, %i, %i, %i, %i, %i,%i, %i, %i, %i,%u, %u, %u, %u, %u, %u, %u,%u, %u, %u, %u, %u, %u, %u, %u,%i, %i,%f, %f, %f",bms.timer, bms.up_time,bms.phase_temp[0],bms.phase_temp[1],bms.phase_temp[2],bms.gate_temp, bms.board_temp, bms.rtd_temp[0],bms.rtd_temp[1],bms.rtd_temp[2],bms.rtd_temp[3],bms.rtd_temp[4],bms.motor_temp,bms.torque_shud, bms.torque_cmd,bms.torque_fb,bms.analog_in[0], bms.analog_in[1], bms.analog_in[2], bms.analog_in[3],bms.digital_in[0], bms.digital_in[1], bms.digital_in[2], bms.digital_in[3], bms.digital_in[4], bms.digital_in[5],bms.motor_angle, bms.motor_speed, bms.inv_freq, bms.resolver_angle,bms.phase_current[0], bms.phase_current[1], bms.phase_current[2], bms.dc_current,bms.dc_voltage, bms.output_volt, bms.p_ab_volt, bms.p_bc_volt,bms.flux_cmd, bms.flux_fb, bms.id_fb, bms.iq_fb, bms.id_cmd, bms.iq_cmd,bms.ref_1_5, bms.ref_2_5, bms.ref_5_0, bms.sys_12v,bms.vsm_state, bms.inv_state, bms.relay_state, bms.inv_mode, bms.inv_cmd, bms.inv_enable, bms.direction,bms.faults[0],bms.faults[1],bms.faults[2],bms.faults[3],bms.faults[4],bms.faults[5],bms.faults[6],bms.faults[7],bms.modulation_index, bms.flux_reg_out,
//   			 s1,s2,out);
        //fprintf(fp, "%f, %f, %f, %f\n", (float)(bms.timer*0.03), s1, s2, out);
        //debuglink.printf("Written\n");
   	 	 	 //timer
   			 //fprintf(fp, "%u, %u",bms.timer, bms.up_time);

   	 	 	 //timer, uptime
   	 fprintf(fp, "%f, %f,"
   			 // phase temp
   			 "%i, %i, %i,"
   			 // temps   rtd temp				motor temp
   			 "%i, %i,  	%i, %i, %i, %i, %i,		%i,"
   			 // torque
   			 "%i, %i, %i,"
   			 // analog in
   			 "%i, %i, %i, %i,"
   			// digitalin
   			 "%u, %u, %u, %u, %u, %u,"
   			 //motor info
   			 "%i, %i, %i, %i,"
   			 //current
   			 "%i, %i, %i, 	%i,"
   			 // Voltage
   			 "%i, %i, %i, %i,"
   			 // Flux
   			 "%i, %i, %i, %i, %i, %i,"
   			 // Internal Voltages
   			 "%i, %i, %i, %i,"
   			 //States
   			 "%u, %u, %u, %u, %u, %u, %u,"
   			 // Faults (8bytes)
   			 "%u, %u, %u, %u, %u, %u, %u, %u,"
   			 //Various
   			 "%i, %i,"
   			 // Throttle input, min cell temp, max cell temp, min cell volt
   			 "%f, %f, %f, %i, %i, %u, %u\n",
   			 (float)bms.timer, (float)bms.up_time/10,
   			 bms.phase_temp[0],bms.phase_temp[1],bms.phase_temp[2],
   			 bms.gate_temp, bms.board_temp, bms.rtd_temp[0],bms.rtd_temp[1],bms.rtd_temp[2],bms.rtd_temp[3],bms.rtd_temp[4],bms.motor_temp,
   			 bms.torque_shud, bms.torque_cmd,bms.torque_fb,
   			 bms.analog_in[0], bms.analog_in[1], bms.analog_in[2], bms.analog_in[3],
   			 bms.digital_in[0], bms.digital_in[1], bms.digital_in[2], bms.digital_in[3], bms.digital_in[4], bms.digital_in[5],
   			 bms.motor_angle, bms.motor_speed, bms.inv_freq, bms.resolver_angle,
   			 bms.phase_current[0], bms.phase_current[1], bms.phase_current[2], bms.dc_current,
   			 bms.dc_voltage, bms.output_volt, bms.p_ab_volt, bms.p_bc_volt,
   			 bms.flux_cmd, bms.flux_fb, bms.id_fb, bms.iq_fb, bms.id_cmd, bms.iq_cmd,
   			 bms.ref_1_5, bms.ref_2_5, bms.ref_5_0, bms.sys_12v,
   			 bms.vsm_state, bms.inv_state, bms.relay_state, bms.inv_mode, bms.inv_cmd, bms.inv_enable, bms.direction,
   			 bms.faults[0],bms.faults[1],bms.faults[2],bms.faults[3],bms.faults[4],bms.faults[5],bms.faults[6],bms.faults[7],
   			 bms.modulation_index, bms.flux_reg_out,
   			 s1,s2,out, bms.min_cell_temp, bms.max_cell_temp, bms.min_cell_volt, bms.max_cell_volt);


//   	 printf(fp, "%u, %u,	%i, %i, %i,		%i, %i,  	%i, %i, %i, %i, %i,		%i, 		%i, %i, %i, 	%i, %i, %i, %i,"
//   			 "%u, %u, %u, %u, %u, %u,"
//   			 "%i, %i, %i, %i,		%i, %i, %i, 	%i,"
//   			 "%i, %i, %i, %i,"
//   			 "%i, %i, %i, %i, %i, %i,%i, %i, %i, %i, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %i, %i, %f, %f, %f");


        fclose(fp);

    }
    else {
   	 //debuglink.printf("Not writing, time=%f\r\n",(float)bms.up_time/10);
    }
}

/**
 * Failsafe check
 *
 * This function will save your life is anything goes wrong.
 * At least in theory - split to different functions at different frequencies
 *
 */
inline void failsafe_periodic(void) {
     // Check if we are ready to drive
	static int rtds_counter;
	if (bms.vsm_state == VSM_ready) {
		if ((rtds_enable == 0) && (rtds_counter < 300)) {
			rtds_enable = 1; // Play RTDS sound
			rtds_counter++;
		}
		else {
			rtds_enable = 0;
		}
	}
	else {
		rtds_counter = 0;
		rtds_enable = 0;
	}

	//throttle readout
	//static float s1;
	//static float s2;
	//static float out;
	s1 = throttle1.read();
	s2 = throttle2.read();

	if (!throttle_plausibility(s1, s2, &out)) {
		acc_out.write(0);
		failsafe_shutdown();
	}
	else {
		acc_out.write(out);
	}

	// brake plausibility
	// if brake ON & throttle > 25% -> shutdown
	// brake ON if low voltage on pin
	/*
	if (brake_en.read() && (out > 0.25)){
		failsafe_shutdown();
	}
	*/

     // check rlecs for faults
     for (int i=0;i<NUM_RLECS;i++){
        if (mlec.rlecsX[i].status == Active) {
            // critical faults
            if ((mlec.rlecsX[i].faults & RLEC_CELL_1_AD_FAULT) != 0) {
                debuglink.printf("!RLEC_CELL_1_AD_FAULT - shutting down...\r\n");  
                failsafe_shutdown();
            }
            else if ((mlec.rlecsX[i].faults & RLEC_CELL_VOLTAGE_CONNECTION_FAULT) != 0) {
                debuglink.printf("!RLEC_CELL_VOLTAGE_CONNECTION_FAULT - shutting down...\r\n");  
                failsafe_shutdown();
            }
            else if ((mlec.rlecsX[i].faults & RLEC_CELL_VOLTAGE_AD_FAULT) != 0) {
                debuglink.printf("!RLEC_CELL_VOLTAGE_AD_FAULT - shutting down...\r\n");  
                failsafe_shutdown();
            }
            else if ((mlec.rlecsX[i].faults & RLEC_MODULE_VOLTAGE_AD_FAULT) != 0) {
                debuglink.printf("!RLEC_MODULE_VOLTAGE_AD_FAULT - shutting down...\r\n");  
                failsafe_shutdown();
            }
            else if ((mlec.rlecsX[i].faults & RLEC_CELL_1_VOLTAGE_FAULT) != 0) {
                debuglink.printf("!RLEC_CELL_1_VOLTAGE_FAULT - shutting down...\r\n");  
                failsafe_shutdown();
            }
            
            // warnings
            else if ((mlec.rlecsX[i].faults & RLEC_CELL_TEMP_AD_FAULT) != 0) {
                debuglink.printf("!RLEC_CELL_TEMP_AD_FAULT - warning light on.\r\n");  
                failsafe_warning();
            }
            else if ((mlec.rlecsX[i].faults & RLEC_RLEC_TEMP_AD_FAULT) != 0) {
                debuglink.printf("!RLEC_RLEC_TEMP_AD_FAULT - warning light on.\r\n");  
                failsafe_warning();
            }
            
            // Charging - overcharge protection
            if (mlec.rlecsX[i].max_cell_volt > MAX_CELL_VOLT) {
                charger_shutdown();
                debuglink.printf("Charging Stopped.\r\n");  
            }
            
            // Voltage limits
            // warning if minimal allowed voltage reached
            if (mlec.rlecsX[i].min_cell_volt < MIN_CELL_VOLT) {
                //batcritical_warning();
            	failsafe_shutdown();
                debuglink.printf("Minimal voltage reached - shutting down.\r\n");
                debuglink.printf("RLEC %i\r\n",i);
                debuglink.printf("Min voltage: %f\r\n",(float)mlec.rlecsX[i].min_cell_volt*0.00244);
            }
            // warning if cell below low threshold
            else if (mlec.rlecsX[i].min_cell_volt < BAT_LOW) {
                //batlow_warning();
                debuglink.printf("Warning - low voltage.\r\n"); 
            }
            // warning if cell below very low threshold
            else if (mlec.rlecsX[i].min_cell_volt < BAT_VERY_LOW) {
                //batverylow_warning();
                debuglink.printf("Warning - very low voltage.\r\n"); 
            }

            //Temperature limits
            if (mlec.rlecsX[i].max_cell_temp > MAX_CELL_TEMP) {
            	failsafe_shutdown();
            	debuglink.printf("Max cell temperature reached- shutting down.\r\n");
            }
        }
    }
}



/*
 * Charger stop (HLIM low)
 * HLIM: 1=ON, 0=OFF
 */
inline void charger_shutdown( void ) {
    hlim.write(0); // HACK: disables HV circuit (as well as charger)
}

/*
 * Open main contactor (LLIM low)
 * LLIM: 1=ON, 0 = FF
 */
inline void failsafe_shutdown( void ) {
    fw_enable = 0; // disable FW_EN

    wait_ms(100); // give some time to remove current from AIRs

	llim = 0; // disable AIR

	hlim.write(0); // HACK: disables HV circuit

	failsafe_warning(); // light up LED
}

/*
 * Light up warning light
 */
inline void failsafe_warning( void ) {
    LED_ON(bms_err_led);
}

/*
 * Light up BatLow
 */
inline void batlow_warning( void ) {
    LED_ON(rtds_enable);
}

/*
 * Light up BatLow & reduce throttle
 */
inline void batverylow_warning( void ) {
    LED_ON(fw_enable);
    // PWM(50%)
}

/*
 * Battery ciritical (i.e. lowest allowed limit)
 */
inline void batcritical_warning( void ) {
    fw_enable = 0; // disable FW_EN

    wait_ms(100); // give some time to remove current from AIRs

    failsafe_shutdown(); // open shutdown circuit
}

/*
 * Ticker callbacks
 */
void failsafe_tid(void){
    flag_failsafe = 1;
}

void telemetry_tid(void){
    flag_telemetry = 1;
}

void downlink_tid(void){
    flag_downlink = 1;
}

void heartbeat_tid(void){
    flag_heartbeat = 1;
}

void can_tid(void){
    flag_can = 1;
}

void datalog_tid(void){
	flag_datalog = 1;
}

/**
 * Throttle plausibility check
 * fsig1 = 5V output
 * fsig2 = 2.5V output
 * scaling factor: 1.9493
 */
bool throttle_plausibility(float sig1, float sig2, float *out) {
	static float acc_out;
	//static bool throttle_ok;
	//static float epsilon;
	//static float fsig1, fsig2;
	static float gain = 1.92;
	// TODO FILTER static float alpha = 0.1;

	//static bool init = false;
	//static int counter;

	// scale stuff up
	sig2 = gain*sig2;

	// moving average filter
	/*
	if (init){
		fsig1 = alpha*sig1 + (1-alpha)*fsig1;
		fsig2 = alpha*sig2 + (1-alpha)*fsig2;
	}
	else {
		fsig1 = sig1;
		fsig2 = sig2;
		init = true;
	}
	*/
	init = true;
	/*
	// calculate the percentage from fsig1 (the larger one)
	epsilon = 0.1 * f_abs(fsig1) + 0.1;
	delta = f_abs(fsig1-fsig2);
	if (((delta) > epsilon) && (counter > 10)) {
		throttle_ok = false;
		acc_out = 0.0;
	}
	else {
		throttle_ok = true;
		acc_out = (fsig1 + fsig2)/2; //mean of two signals
	}
	*/

	throttle_ok = true;
	static float a = 0.1;
	static float b = 1.0;
	static float min = 0.12;
	static float max = 0.45;

	acc_out = ((b-a)*(sig1-min)/(max-min))+a;

	counter++;
	*out = acc_out;
	return throttle_ok;
}


inline float f_abs(float val) {
	if (val >= 0) {
		return val;
	}
	else {
		return -val;
	}
}
