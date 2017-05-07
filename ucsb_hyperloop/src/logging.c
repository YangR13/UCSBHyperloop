#include "logging.h"
#include "subsystems.h"
#include "ethernet.h"
#include "sensor_data.h"
#include "rtc.h"
#include "I2CPERIPHS.h"

void get_filepath(char* filepath, char* dir, LOG_TYPE log_type, int index, char* filetype) {
	// Concat the dir and name variables and save to the filepath variable.
	if(strcmp("", dir) == 0) {
		sprintf(filepath, "%s_%d.%s", LOG_TYPE_STRINGS[log_type], index, filetype);
	}
	else {
		sprintf(filepath, "%s/%s_%d.%s", dir, LOG_TYPE_STRINGS[log_type], index, filetype);
	}
}

FRESULT f_open_log (FIL *fp, LOG_TYPE log_type, int index, BYTE mode)
{
	FRESULT rc;
	char filepath[32] = "";
	get_filepath(filepath, g_log_directory, log_type, index, TYPE_CSV);
	rc = f_open(fp, filepath, mode);
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_open_log rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

FRESULT f_write_log(FIL *fp, LOG_TYPE log_type, int index, char* data)
{
	FRESULT rc;
	UINT bw;
	char data_[17];
	snprintf(data_, 17, "%s,", data);
	rc = f_write(fp, data_, strlen(data_), &bw);
	LOG_POSITIONS[log_type][index] += bw;
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_write_log rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

FRESULT f_write_newline(FIL *fp, LOG_TYPE log_type, int index)
{
	FRESULT rc;
	UINT bw;
	rc = f_write(fp, "\r\n", 2, &bw);
	LOG_POSITIONS[log_type][index] += bw;
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_write_newline rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

void init_csv_files() {
	int i, j;

	/* Get local time */
	RTC rtc;
	rtc_gettime(&rtc);

	// Create a new log directory for this session. Directory names limited to 8 characters.
	snprintf(g_log_directory, 8, "%02d-%02d-%02d", rtc.hour, rtc.min, rtc.sec);
	f_mkdir(g_log_directory);

	// Initialize array for keeping track of file positions.
	for(i=0; i<NUM_LOGS; i++) {
		for(j=0; j<6; j++) {
			LOG_POSITIONS[i][j] = 0;
		}
	}

	// Create new files and add headers.
	create_csv(g_log_directory, LOG_POSITION, 0);

	for(i=0; i<4; i++) {
		create_csv(g_log_directory, LOG_HEMS, i);
	}
	for(i=0; i<6; i++) {
		create_csv(g_log_directory, LOG_MAGLEV_BMS, i);
	}

	logData(LOG_HEMS, 0);
}

void create_csv(char* dir, LOG_TYPE log_type, int index)
{
	FIL fileObj;	/* File object */
	FRESULT rc;
	UINT bw;
	f_open_log (&fileObj, log_type, index, FA_WRITE | FA_CREATE_ALWAYS);
	// Add headers.
	char header[128] = "";
	switch(log_type) {
	case LOG_POSITION:
		snprintf(header, 128, "Time,X-Pos,X-Vel,X-Accel,Y-Pos,Y-Vel,Y-Accel,Z-Pos,Z-Vel,Z-Accel,Roll,Pitch,Yaw,Contact\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	case LOG_HEMS:
		snprintf(header, 128, "Time,DAC,Current,RPM,Temp 0,Temp 1,Temp 2,Temp 3\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	case LOG_MAGLEV_BMS:
		snprintf(header, 128, "Time,B0 Low,B0 High,B0 Temp 0,B0 Temp 1,B1 Low,B1 High,B1 Temp 0,B1 Temp 1,Batt 2 Low,Batt 2 High,B2 Temp 0,B2 Temp 1\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	default:
		break;
	}
	// Update file position.
	LOG_POSITIONS[log_type][index] += bw;

	if(rc != 0) DEBUGOUT("ERROR: %s f_write rc=%d\n", LOG_TYPE_STRINGS[log_type], rc);
	f_close(&fileObj);
}

void logAllData(){
	int i;

	if(RANGING_SENSORS_ACTIVE) {
		ethernet_prepare_packet();
		logData(LOG_POSITION, 0);
		if((sendDataFlag && connectionOpen)) ethernet_send_packet();
	}

	if(MOTOR_BOARD_I2C_ACTIVE) {
		for(i=0; i<4; i++) {
			ethernet_prepare_packet();
			logData(LOG_HEMS, i);
			if((sendDataFlag && connectionOpen)) ethernet_send_packet();
		}
	}
	if(MAGLEV_BMS_ACTIVE) {
		for(i=0; i<2; i++) {
			ethernet_send_packet();
			logData(LOG_MAGLEV_BMS, i);
			if((sendDataFlag && connectionOpen)) ethernet_send_packet();
		}
	}
}

void logData(LOG_TYPE log_type, int index)
{
	FIL fileObj;	/* File object */
	FRESULT rc;
	UINT bw;
	int i, j;

	f_open_log (&fileObj, log_type, index, FA_WRITE);

	// Seek current position in log file.
	f_lseek(&fileObj, LOG_POSITIONS[log_type][index]);

	char data[16] = "";
	switch(log_type) {
	case LOG_POSITION:
		// Time
		// ???

		// Photoelectric
		snprintf(data, 16, "%06.2f", sensorData.photoelectric);
		//ethernet_add_data_to_packet(PH, -1, -1, data);
		//rc = f_write_log(&fileObj, log_type, index, data);

		// Short Ranging
		for(i=0; i<4; i++) {
			snprintf(data, 16, "%06.2f", motors[i]->short_data[0]);
			ethernet_add_data_to_packet(SR, i, -1, data);
			//rc = f_write_log(&fileObj, log_type, index, data);
		}

		// Newline
		//rc = f_write_newline(&fileObj, log_type, index);

		break;
	case LOG_HEMS:
		// Time
		snprintf(data, 16, "%06.2f", motors[index]->timestamp);
		// ethernet_add_data_to_packet
		rc = f_write_log(&fileObj, log_type, index, data);

		// DAC
		if(index == 0) {
			snprintf(data, 16, "%06.2f", motors[index]->throttle_voltage);
			ethernet_add_data_to_packet(DAC, -1, -1, data);
		}
		rc = f_write_log(&fileObj, log_type, index, data);

		// Current
		snprintf(data, 16, "%06.2f", (float)motors[index]->amps);
		ethernet_add_data_to_packet(CU, index, -1, data);
		rc = f_write_log(&fileObj, log_type, index, data);

		// RPM
		snprintf(data, 16, "%06.2f", (float)motors[index]->rpm[1]);
		ethernet_add_data_to_packet(TA, index, -1, data);
		rc = f_write_log(&fileObj, log_type, index, data);

		// Temperature (0 to 3)
		int i;
		for(i=0; i<4; i++){
			snprintf(data, 16, "%06.f", (float)motors[index]->temperatures[i]);
			ethernet_add_data_to_packet(TM, index, i, data);
			rc = f_write_log(&fileObj, log_type, index, data);
		}

		// Newline
		rc = f_write_newline(&fileObj, log_type, index);
		break;
	case LOG_MAGLEV_BMS:
		// BMS(index) Voltages
		for(j=0; j<3; j++){
			for(i=0; i<6; i++){
				snprintf(data, 16, "%06.2f", maglev_bmses[index]->cell_voltages[j][i]);
				ethernet_add_data_to_packet(BMSV, index, (6*j)+i, data);
			}
		}
		// BMS(index) Temperature
		for(j=0; j<3; j++){
			for(i=0; i<2; i++){
				snprintf(data, 16, "%06.2f", maglev_bmses[index]->temperatures[j][i]);
				ethernet_add_data_to_packet(BMST, index, (2*j)+i, data);
			}
		}
		break;
	default:
		break;
	}

	f_close(&fileObj);

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
