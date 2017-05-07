#include "logging.h"
#include "subsystems.h"
#include "ethernet.h"
#include "sensor_data.h"
#include "rtc.h"
#include "I2CPERIPHS.h"

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
	FRESULT rc;
	int i, j;

	rc = f_open_log(log_type, index, FA_WRITE);

	// Seek current position in log file.
	rc = f_lseek_(LOG_POSITIONS[log_type][index]);

	char data[16] = "";
	switch(log_type) {
	case LOG_POSITION:
		// Time
		// ???

		// Photoelectric
		snprintf(data, 16, "%06.2f", sensorData.photoelectric);
		//ethernet_add_data_to_packet(PH, -1, -1, data);
		//rc = f_write_log(log_type, index, data);

		// Short Ranging
		for(i=0; i<4; i++) {
			snprintf(data, 16, "%06.2f", motors[i]->short_data[0]);
			ethernet_add_data_to_packet(SR, i, -1, data);
			//rc = f_write_log(log_type, index, data);
		}

		// Newline
		//rc = f_write_newline(log_type, index);

		break;
	case LOG_HEMS:
		// Time
		snprintf(data, 16, "%06.2f", motors[index]->timestamp);
		// ethernet_add_data_to_packet
		rc = f_write_log(log_type, index, data);

		// DAC
		if(index == 0) {
			snprintf(data, 16, "%06.2f", motors[index]->throttle_voltage);
			ethernet_add_data_to_packet(DAC, -1, -1, data);
		}
		rc = f_write_log(log_type, index, data);

		// Current
		snprintf(data, 16, "%06.2f", (float)motors[index]->amps);
		ethernet_add_data_to_packet(CU, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// RPM
		snprintf(data, 16, "%06.2f", (float)motors[index]->rpm[1]);
		ethernet_add_data_to_packet(TA, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Temperature (0 to 3)
		int i;
		for(i=0; i<4; i++){
			snprintf(data, 16, "%06.f", (float)motors[index]->temperatures[i]);
			ethernet_add_data_to_packet(TM, index, i, data);
			rc = f_write_log(log_type, index, data);
		}

		// Newline
		rc = f_write_newline(log_type, index);
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

	f_close_();
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
	strcat(desc, control_signal_names[sig]);
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
