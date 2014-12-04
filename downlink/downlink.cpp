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
 * @file downlink/downlink.c
 *
 * Telemetry for BMS
 */
#include "downlink.h"
#include "SDFileSystem.h"
#include <ctype.h>
#include <string>

#include "main.h"

AnalogOut acc_out(p18);
AnalogIn throttle1(p16); //s1
AnalogIn throttle2(p17); //s2

#define GEAR_RATIO 3.0

AnalogIn charge_current(p15);

Serial debuglink(USBTX, USBRX);
//Serial debuglink(p13, p14);
Serial torquelink(p28, p27); 
Serial telemetrylink(p13, p14);
//Serial telemetrylink(USBTX, USBRX);
SDFileSystem sd(p5, p6, p7, p8, "sd");
FILE *fp;

enum DownlinkState downlink_status;
static char rx_buf[DOWNLINK_BUFFER_SIZE];
static char tx_buf[TELEMETRY_BUFFER_SIZE];
static uint8_t d_idx;
bool bLineFeed;
bool bHeader;
bool bEcho;
bool bHeaders;

int file_counter;

time_t f_seconds;
char logname[32];

string toHexString(uint8_t f)
{
	ostringstream oss;
	oss << std::uppercase << std::hex << (int)f;
	return oss.str();
}

string toHexString(uint16_t f)
{
	ostringstream oss;
	oss << std::uppercase << std::hex << (int)(f >> 8) << " " << (int)(f & 0xff);
	return oss.str();
}

/**
 * Bound number (int16_t)
 */
int16_t boundNumberShort(int16_t num, int16_t lower, int16_t upper){
	if (num > upper) {
		num = upper;
		return num;
	}
	if (num < lower) {
		num = lower;
		return num;
	}
	else {
		return num;
	}
}

/**
 * Bound number (uint16_t)
 */
uint16_t boundNumberUnsigShort(uint16_t num, uint16_t lower, uint16_t upper){
	if (num > upper) {
		num = upper;
		return num;
	}
	if (num < lower) {
		num = lower;
		return num;
	}
	else {
		return num;
	}
}

/**
 * Bound number (uint8_t)
 */
uint8_t boundNumberUnsigByte(uint8_t num, uint8_t lower, uint8_t upper){
	if (num > upper) {
		num = upper;
		return num;
	}
	if (num < lower) {
		num = lower;
		return num;
	}
	else {
		return num;
	}
}

/**
 * Downlink init
 */
void downlink_init(void){
	// Torque init
	torquelink.baud(DOWNLINK_BAUDRATE);
	torquelink.printf("I am alive!\n");
	downlink_status = DataMode;
	memset(rx_buf, 0, DOWNLINK_BUFFER_SIZE);
	d_idx = 0;
	bLineFeed = true;
	bHeader = false;
	bEcho = true;
	bHeaders = false;

	// Debug link init
	debuglink.baud(DEBUGLINK_BAUDRATE);
	debuglink.printf("I am alive!\r\n");

	// Telemetry init
	telemetrylink.baud(TELEMETRY_BAUDRATE);
	memset(tx_buf, 0, TELEMETRY_BUFFER_SIZE);
	torquelink.printf("Telemetry initalized.\n");

	// SD card init
	f_seconds = time(NULL);
	strftime(logname, 32, "/sd/%F_%H-%M-%S.txt", localtime(&f_seconds));

	fp = fopen(logname, "w");
	if(fp != NULL) {
		debuglink.printf("SD card counter opened!\n");
		//fprintf(fp, "Timer, Throttle1, Throttle2, Out\n");
		//rewind(fp);
		fprintf(fp, "Timer, Uptime, PhaseTemp1, PhaseTemp2, PhaseTemp3, GateTemp, BoardTemp, RtdTemp1, RtdTemp2, RtdTemp3, RtdTemp4, RtdTemp5,"
				"MotorTemp, TorqueShud, TorqueCmd, TorqueFb, AnIn1, AnIn2, AnIn3, AnIn4, DigIn1, DigIn2, DigIn3, DigIn4, DigIn5, DigIn6, "
				"MotorAngle, MotorSpeed, InvFreq, ResAng, PhasCur1, PhasCur2, PhasCur3, DcCur, DcVolt, OutputVolt, PabVolt, PbcVolt,"
				"FluxCmd, FluxFb, IdFb, IqFb, IdCmd, IqCmd, Ref15, Ref25, Ref50, Sys12V, VSMstate, InvState, RelayState, InvMode, InvCmd, InvEn,"
				"Direction, Fault1, Fault2, Fault3, Fault4, Fault5, Fault6, Fault7, Fault8, ModIdx, FluxRegOut, ThrottleIn1, ThrottleIn2, ThrottleOut, "
				"MinCellTemp, MaxCellTemp, MinCellVolt, MaxCellVolt"
				"\n");
		fclose(fp);
	}
	else {
		debuglink.printf("SD card error!\r\n");
	}

	LED_ON(led_telemetry);
}

/**
 * Telemetry periodic
 */
void downlink_periodic(void){
	LED_TOGGLE(led_telemetry);

	// Debug
	//debuglink.printf("Uptime: %f sec.\r\n", (float)bms.up_time/10);
	bms.max_cell_temp = 0;
	bms.min_cell_temp = 100;
	bms.max_cell_volt = 0;
	bms.min_cell_volt = -1;

	for (int i=0;i<NUM_RLECS;i++){
		if (mlec.rlecsX[i].status == Active) {
			//debuglink.printf(">>> RLEC_ %i\r\n", i);
			//debuglink_print_rlec(&(mlec.rlecsX[i]));
			//debuglink.printf("\r\n");

			// check max cell temp
			if (mlec.rlecsX[i].max_cell_temp > bms.max_cell_temp) {
				bms.max_cell_temp = mlec.rlecsX[i].max_cell_temp;
			}

			// check min cell temp
			if (mlec.rlecsX[i].min_cell_temp < bms.min_cell_temp) {
				bms.min_cell_temp = mlec.rlecsX[i].min_cell_temp;
			}

			// check max cell voltage
			if (mlec.rlecsX[i].max_cell_volt > bms.max_cell_volt) {
				bms.max_cell_volt = mlec.rlecsX[i].max_cell_volt;
			}

			// check min cell voltage
			if (mlec.rlecsX[i].min_cell_volt < bms.min_cell_volt) {
				bms.min_cell_volt = mlec.rlecsX[i].min_cell_volt;
			}
		}
	}
	//debuglink.printf("Min cell tmp %i\r\n",bms.min_cell_temp);
	//debuglink.printf("Max cell tmp %i\r\n",bms.max_cell_temp);
	//debuglink.printf("Min cell volt %f\r\n",(float)bms.min_cell_volt*RLEC_CAN_VOLTAGE_MULT);
	//debuglink.printf("Max cell volt %f\r\n",(float)bms.max_cell_volt*RLEC_CAN_VOLTAGE_MULT);
	//debuglink.printf("\r\n");

	// Print BMS values
	/*
	debuglink.printf("DC voltage: %i [V]\r\n", bms.dc_voltage/10);
	debuglink.printf("DC current: %i [A]\r\n", bms.dc_current/10);
	debuglink.printf("battery voltage: %i [V]\r\n", bms.sys_12v/100);
	debuglink.printf("board_temp: %i [C]\r\n", bms.board_temp/10);
	debuglink.printf("vsm_state: %i \r\n", bms.vsm_state);
	debuglink.printf("inv_state: %i \r\n", bms.inv_state);
	debuglink.printf("HLIM: %i \r\n", hlim.read());
	debuglink.printf("\r\n");
	debuglink.printf("\r\n");
	debuglink.printf("\r\n");
	*/

	// send telemetry over USB cable
	send_data_over_usb();
}

void send_data_over_usb(void) {
    float f_timer = (float)bms.timer*0.003;
    float f_sys_time = (float)bms.up_time/100;
	debuglink.printf("%f, %f,"
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
  			 f_timer, f_sys_time,
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
}


/**
 * Telemetry periodic
 */
void telemetry_periodic(void){
	// Telenetry
	static uint8_t cksum0, cksum1;
	static uint16_t idx;

	tx_buf[0] = TELEMETRY_MSG0;
	tx_buf[1] = TELEMETRY_MSG1;

	cksum0 = 0;
	cksum1 = 0;

	tx_buf[2] = 0;
	tx_buf[3] = 0;
	idx = TELEMETRY_DATA_IDX;

	// put int data

	//uptime uint16_t
	//uint16_t xxx = 0xABCD;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.up_time>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// DC current - dc_current, int16_t
	//xxx = 0x1234;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.dc_current>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// RPM - motor_speed, int16_t
	//xxx = 0x1A2A;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.motor_speed>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// DC Voltage - dc_voltage, int16_t
	//xxx = 0xACDC;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.dc_voltage>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// torque_fb - estimated motor torque, int16_t
	//xxx = 0x4554;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.torque_fb>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// min cell temp
	tx_buf[idx] = bms.min_cell_temp;
	idx++;

	// max cell temp
	tx_buf[idx] = bms.max_cell_temp;
	idx++;

	// min cell voltage, uint16_t
	//xxx = 0x4554;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.min_cell_volt>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// max cell voltage, uint16_t
	//xxx = 0x4554;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[idx] = 0xFF&(bms.max_cell_volt>>(i*8));
		//tx_buf[idx] = 0xFF&(xxx>>(i*8));
		idx++;
	}

	// fill in data length, uint16
	uint16_t n = idx - TELEMETRY_DATA_IDX;
	for (uint8_t i = 0; i<2; i++){
		tx_buf[TELEMETRY_SIZE_IDX+i] = 0xFF&(n>>(i*8));
	}

	// calculate checksum & send
	static uint8_t c0, c1;
	static uint16_t i;//, size;
	c0 = 0;
	c1 = 0;

	//Start at byte three so MSG0 and MSG1 is not part of the checksum
	for ( i = TELEMETRY_DATA_IDX-2; i < idx; i++ ) {
		c0 += (uint8_t)tx_buf[i];
		c1 += c0;
	}
	cksum0 = c0;
	cksum1 = c1;

	tx_buf[idx] = cksum0;
	idx++;
	tx_buf[idx] = cksum1;
	idx++;

	// transmit
	for (uint16_t k = 0; k<idx; k++){
		telemetrylink.putc(tx_buf[k]);
	}
}

/**
 * Datalink event
 */
void downlink_event(void){
	static char c;

	// USB serial
	if(debuglink.readable()) {
		c = debuglink.getc();
		/*
		if (c == 'a') mlec.rlecsX[15].faults = 0x1;
		if (c == 'b') mlec.rlecsX[15].faults = 0x2;
		if (c == 'c') mlec.rlecsX[15].faults = 0x4;
		if (c == 'd') mlec.rlecsX[15].faults = 0x8;
		if (c == 'e') mlec.rlecsX[15].faults = 0x10;
		if (c == 'f') mlec.rlecsX[15].faults = 0x20;
		if (c == 'g') mlec.rlecsX[15].faults = 0x40;
		if (c == 'h') mlec.rlecsX[15].faults = 0x80;
		if (c == 'i') mlec.rlecsX[15].faults = 0x0;
		*/
		switch (c) {
			case 'a':
				hlim.write(0);
				break;
			case 'b':
				mlec.rlecsX[15].faults = 0x1;
				break;
			default:
				break;
		}
		debuglink.printf(">%c\n", c);
	}

	// Torque
	if(torquelink.readable()) {
		downlink_parse(torquelink.getc());
	}
}

void downlink_parse(char c) {
	// parse until we get 0xD <CR>
	//debuglink.printf(">: %c\n", c);
	if ((c == 0xD) || (d_idx > DOWNLINK_BUFFER_SIZE)) {
		rx_buf[d_idx] = 0; //null terminate the string
		d_idx = 0;
		downlink_process_cmd();
	}
	else {
		if ((c != 0xA) && (c != 0x20)) { // no LF, no Space
			rx_buf[d_idx++] = (char)tolower(c);
		}
	}
}

void downlink_process_cmd(void) {
	string retString = string();
	string lineEnding;
	if (bLineFeed) lineEnding = string("\r\n>");
	else lineEnding = string("\r>");
	if (!strncmp(rx_buf, "at", 2)) {
		if (!strcmp(rx_buf, "ati")) { // print the version ID
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT I\r\r\n");
			retString+="\rELM327 v1.3a";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "at@1")) { // display the device description
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT @1\r\r\n");
			retString+="\reVCU 1.0";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atdpn")) { // Describe the Protocol by Number
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT DPN\r\r\n");
			retString+="\r6";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atdp")) { // Describe the current Protoco
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT DP\r\r\n");
			retString+="\rISO 15765-4 CAN (11 bit ID, 500 kbaud)";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atrv")) { // Read the input Voltage
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT RV\r\r\n");
			// GLV value (12V battery voltage - analog input)
			retString+="\r 12.3V";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atl0")) { //turn linefeeds off
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT L0\r\r\n");
			bLineFeed = false;
			retString+=("\rOK");
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atl1")) { //turn linefeeds on
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT L1\r\r\n");
			bLineFeed = true;
			retString+=("\rOK");
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "atz")) { //reset hardware
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT Z\r\r\n");
			retString+="\rELM327 v1.3a";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "ath0")) { // headers off
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT H0\r\r\n");
			bHeaders = false;
			retString+=("\rOK");
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "ath1")) { //turn headers on
			debuglink.printf("Command received: %s\n", rx_buf);
			if (bEcho) retString+=("AT H1\r\r\n");
			bHeaders = true;
			retString+=("\rOK");
			retString+=lineEnding;
		}
		else {
			debuglink.printf("Unknwon AT command: %s\n", rx_buf);
			retString+="\rOK";
			retString+=lineEnding;
		}
	}
	else { // ODB request
		if (!strcmp(rx_buf, "0100")) { // available sensors
			debuglink.printf("Command received: %s\n", rx_buf);
			retString+=  "4100 08 1A 80 02";
			//retString+="4100 08 3A 00 00";
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "0105")) { // Engine coolant temperature (Motor temp)
			retString+="41 05 ";// 70C = 158F (1 	Engine coolant temperature 	-40 	215 	°C 	A-40)
			uint8_t tmp = (uint8_t)boundNumberShort(bms.motor_temp/10+40, 0, 255);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "010c")) { // Engine RPM (the received value is divided by 4)
			// 0C 	2 	Engine RPM 	0 	16,383.75 	rpm 	((A*256)+B)/4
			retString+="41 0C ";//retString+="41 0C 68 28";// 6666RPM
			uint16_t tmp = boundNumberUnsigShort(bms.motor_speed*4, 0, 65535);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "010d")) { // Vehicle speed (kmh)
			//0D 	1 	Vehicle speed 	0 	255 	km/h 	A
			//debuglink.printf("Command received: %s\n", rx_buf);
			retString+="41 0D ";//retString+="41 0D 88";//85 mph
			uint16_t speed = (uint16_t)((float)bms.motor_speed*GEAR_RATIO*0.06);
			uint8_t tmp = (uint8_t)boundNumberShort(speed, 0, 255);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "010f")) { // Intake air temperature (Controller temperature)
			//debuglink.printf("Command received: %s\n", rx_buf);
			//0F 	1 	Intake air temperature 	-40 	215 	°C 	A-40
			retString+="41 0F ";//retString+="41 0F 46";// 70C = 158F
			uint8_t tmp = (uint8_t)boundNumberShort(bms.board_temp/10+40, 0, 255);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "0111")) { // Throttle position (throttle encoder)
			//debuglink.printf("Command received: %s\n", rx_buf);
			//11 	1 	Throttle position 	0 	100 	 % 	A*100/255
			float throttle = acc_out.read()*255;
			uint8_t tmp = (uint8_t)throttle;
			retString+="41 11 ";//retString+="41 11 6B";// 42 %
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "011c")) { // OBD standards this vehicle conforms to
			//debuglink.printf("Command received: %s\n", rx_buf);
			retString+="41 1C 02";// 2 - OBD as defined by the EPA
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "011f")) { // Run time since engine start (uptime)
			//debuglink.printf("Command received: %s\n", rx_buf);
			//1F 	2 	Run time since engine start 	0 	65,535 	seconds 	(A*256)+B
			retString+="41 1F ";//retString+="41 1F 00 0D";// 13s
			uint16_t tmp = boundNumberUnsigShort(bms.up_time, 0, 65535);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "01c3")) { // HV Voltage
			//45 300 	V 	A+40
			retString+="41 C3 ";//retString+="41 C3 91";// 290V
			uint8_t tmp = (uint8_t)boundNumberShort(bms.dc_voltage/10-40, 0, 255);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else if (!strcmp(rx_buf, "01c4")) { // HV Current
			// 0-6 553.5 A 	((A*256)+B)/10
			retString+="41 C4 ";//retString+="41 C4 28";// 30A
			uint16_t tmp = boundNumberUnsigShort(bms.dc_current, 0, 65535);
			retString+=toHexString(tmp);
			retString+=lineEnding;
		}
		else {
			//unknown command
			debuglink.printf("Unknown OBD command: %s\n",rx_buf);
		}
	}
	torquelink.printf(retString.c_str());
	//debuglink.printf("Command responded: %s\n",retString.c_str());
}

void debuglink_print_rlec(struct RLECModule* rlec) {
	debuglink.printf("max_cell_volt: %f [V]\r\n", (float)(rlec->max_cell_volt*RLEC_CAN_VOLTAGE_MULT));
	debuglink.printf("min_cell_volt: %f [V]\r\n", (float)(rlec->min_cell_volt*RLEC_CAN_VOLTAGE_MULT));
	debuglink.printf("rlec_temp: %i [C]\r\n", rlec->rlec_temp);
	debuglink.printf("balance_resistors: %X\r\n", rlec->balance_resistors);
	debuglink.printf("faults: %X\r\n", rlec->faults);
	debuglink.printf("rlec_volt: %f [V]\r\n", (float)(rlec->rlec_volt*RLEC_CAN_MODULE_MULT));
	debuglink.printf("max_cell_temp: %i [C]\r\n", rlec->max_cell_temp);
	debuglink.printf("min_cell_temp: %i [C]\r\n", rlec->min_cell_temp);
}
