#include "logging.h"
#include "subsystems.h"
#include "ethernet.h"
#include "I2CPERIPHS.h"
#include "sensor_data.h"
#include "sdcard.h"
#include "timer.h"

void get_filepath(char* filepath, char* dir, char* name, int index) {
	// Concat the dir and name variables and save to the filepath variable.
}

void initSDCard() {
	// Create a new directory for this session.
	char path[32];


	// Create new files and add headers.
	create_position_csv();
	int i;
	for(i=0; i<4; i++) {
		create_hems_csv(i);
	}

}

void create_csv(char* dir, char* filetype, int index)
{
	FIL fileObj;	/* File object */
	char filepath[32];
	get_filepath(filepath, dir, filetype, index);
	f_open(&fileObj, filepath, FA_WRITE | FA_CREATE_ALWAYS);

	// Add headers.
	if(strcmp(filetype, FILE_POSITION) == 0) {

	}
	if(strcmp(filetype, FILE_HEMS) == 0) {

	}
	if(strcmp(filetype, FILE_BMS) == 0) {

	}

	f_close(&fileObj);
}

void logData(){
    logDataFlag = 0;

	int i;

	if(RANGING_SENSORS_ACTIVE) {
		ethernet_prepare_packet();
		for(i=0; i<4; i++){
			logPosition(i);
		}
		if(connectionOpen)
			ethernet_send_packet();
	}

	if(MOTOR_BOARD_I2C_ACTIVE) {
		ethernet_prepare_packet();
		for(i=0; i<4; i++) {
			logHEMS(i);
		}
		if(connectionOpen)
			ethernet_send_packet();
	}

	if(MAGLEV_BMS_ACTIVE){
		for(i=0; i<2; i++){
			ethernet_prepare_packet();
			logBMS(i);
			if(connectionOpen)
				ethernet_send_packet();
		}
	}
}

void logPosition(index){

	char data[16];
//	snprintf(data, 16, "%06.2f", sensorData.photoelectric);
//	ethernet_add_data_to_packet(PH, -1, -1, data);

	// Short Ranging
	snprintf(data, 16, "%06.2f", motors[index]->short_data[0]);
	ethernet_add_data_to_packet(SR, index, -1, data);
}

void logHEMS(int index){
	char data[16];
	// DAC
	if(index == 0) {
		snprintf(data, 16, "%06.2f", motors[index]->throttle_voltage);
		ethernet_add_data_to_packet(DAC, -1, -1, data);
	}

	// Current
	snprintf(data, 16, "%06.2f", (float)motors[index]->amps);
	ethernet_add_data_to_packet(CU, index, -1, data);

	// RPM
	snprintf(data, 16, "%06.2f", (float)motors[index]->rpm[1]);
	ethernet_add_data_to_packet(TA, index, -1, data);

	// Temperature (0 to 3)
	int i;
	for(i=0; i<4; i++){
		snprintf(data, 16, "%06.f", (float)motors[index]->temperatures[i]);
		ethernet_add_data_to_packet(TM, index, i, data);
	}

}

void logBMS(int index){
	char data[16];
	memset(data, 0, 16);
	// BMS(index) Voltages
	int i, j;
	for(j=0; j<3; j++){
		for(i=0; i<6; i++){
			snprintf(data, 16, "%06.2f", maglev_bmses[index]->cell_voltages[j][i]);
			if(j>0){
				ethernet_add_data_to_packet(BMSV, index, (6*j)+i, data);
			}
			else{
				ethernet_add_data_to_packet(BMSV, index, i, data);
			}
		}
	}
	// BMS(index) Temperature
	for(j=0; j<3; j++){
		for(i=0; i<2; i++){
			snprintf(data, 16, "%06.2f", maglev_bmses[index]->temperatures[j][i]);
			if(j>0){
				ethernet_add_data_to_packet(BMST, index, (2*j)+i, data);
			}
			else{
				ethernet_add_data_to_packet(BMST, index, i, data);
			}
		}
	}
}


void initEventLogFile()
{

}

void initErrorLogFile()
{

}

void logEventString(char* desc)
{
	// Send to web app.
	// Copy strings to Net_Tx_Data
//	int pos = 0;
//	memset(Net_Tx_Data, 0, 512); // Make sure this clears enough space

	/* DAC Output */
//	char errorMsg[512];

//	sprintf(errorMsg, "%s", desc);

	/* DAC Data */
//	send_data_packet_helper(DAC, errorMsg, &pos);

//	Wiz_Send_Blocking(SOCKET_ID, Net_Tx_Data);
	// Save to SD card file.

}

void logErrorString(char* desc)
{
	// Send to web app.

	// Save to SD card file.

}

void logStateMachineEvent(int sig)
{
	char desc[64] = "Control signal: ";
	char *sig_desc;
	sig_desc = control_signal_names[sig];
	strcat(desc, sig_desc);
	logEventString(desc);
}

void logEvent(Event evt)
{
	char desc[64] = "Event: ";
	char *evt_desc;
	switch(evt)
	{
		// case ???:				evt_desc = "???";					break;
		default:					evt_desc = "UNKNOWN_EVT";			break;
	}
	strcat(desc, evt_desc);
	logEventString(desc);
}

void logError(Error err)
{
	char desc[64] = "Error: ";
	char *err_desc;
	switch(err)
	{
		// case ???:				err_desc = "???";					break;
		default:					err_desc = "UNKNOWN_ERR";			break;
	}
	strcat(desc, err_desc);
	logEventString(desc);
}
