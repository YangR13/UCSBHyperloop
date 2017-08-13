#include "logging.h"
#include "subsystems.h"
#include "ethernet.h"
#include "sensor_data.h"
#include "rtc.h"
#include "I2CPERIPHS.h"
#include "sdcard.h"
#include "timer.h"

void logAllData(){
	if(!GATHER_DATA_ACTIVE) {
		return;
	}
	ethernet_prepare_packet();

	if(POSITIONING_ACTIVE) {
		logData(LOG_POSITIONING_NAVIGATION);
	}
	if(MOTOR_BOARD_I2C_ACTIVE || MAGLEV_BMS_ACTIVE) {
		logData(LOG_MAGLEV);
	}

	if(BRAKING_ACTIVE) {
		logData(LOG_BRAKING);
	}

	if(PAYLOAD_ACTUATORS_ACTIVE || SERVICE_PROPULSION_ACTIVE) {
		logData(LOG_SERV_PROP_PAYLOAD);
	}
	ethernet_send_packet();

}
void logData(LOG_TYPE log_type) {
	char data[128] = "";
	int i, j;
	float v_low[4];
	float v_high[4];
	int t_high[4];

	switch(log_type) {
	case LOG_POSITIONING_NAVIGATION:
		// Position X, Position Y, Position Z.
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f",
			sensorData.positionX, sensorData.positionY, sensorData.positionZ);
		ethernet_add_data_to_packet(POS, -1, -1, data);

		// Velocity X, Velocity Y, Velocity Z.
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f",
			sensorData.velocityX, sensorData.velocityY, sensorData.velocityZ);
		ethernet_add_data_to_packet(VEL, -1, -1, data);

		// Acceleration X, Acceleration Y, Acceleration Z.
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f",
			sensorData.accelX, sensorData.accelY, sensorData.accelZ);
		ethernet_add_data_to_packet(ACL, -1, -1, data);

		// Roll, Pitch, Yaw.
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f",
			sensorData.roll, sensorData.pitch, sensorData.yaw);
		ethernet_add_data_to_packet(RPY, -1, -1, data);

		break;

	case LOG_POWER_DISTRIBUTION:
		// Power Distribution BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low).
		for(i=0; i<2; i++) {
			v_low[i] = -1;
			v_high[i] = -1;
		}
		for(i=0; i<2; i++) {
			for(j=0; j<5; j++){
				float v = pwr_dst_bms->cell_voltages[i][j];
				if(v_low[i] == -1 || v < v_low[i]) v_low[i] = v;
				if(v_high[i] == -1 || v > v_high[i]) v_high[i] = v;
			}
		}
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			v_high[0], v_low[0], v_high[1], v_low[1]);
		ethernet_add_data_to_packet(PD_BMS_VHL, -1, -1, data);

		// Power Distribution BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent).
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			pwr_dst_bms->battery_charge_coulomb[0], pwr_dst_bms->battery_charge_percent[0],
			pwr_dst_bms->battery_charge_coulomb[1], pwr_dst_bms->battery_charge_percent[1]);
		ethernet_add_data_to_packet(PD_BMS_CHG, -1, -1, data);

		// Power Distribution BMS Battery0 Temperature (High), Battery1 Temperature (High).
		for(i=0; i<2; i++) {
			int t0 = pwr_dst_bms->temperatures[i][0];
			int t1 = pwr_dst_bms->temperatures[i][1];
			t_high[i] = (t0 > t1) ? t0 : t1;
		}
		snprintf(data, 128, "%d, %d",
			t_high[0], t_high[1]);
		ethernet_add_data_to_packet(PD_BMS_TH, -1, -1, data);

		break;

	case LOG_MAGLEV:
		// Maglev0 RPM, Maglev1 RPM, Maglev2 RPM, Maglev3 RPM.
		snprintf(data, 128, "%d, %d, %d, %d",
			motors[0]->rpm[1], motors[1]->rpm[1], motors[2]->rpm[1], motors[3]->rpm[1]);
		ethernet_add_data_to_packet(M_RPM, -1, -1, data);

		// Maglev0 Current, Maglev1 Current, Maglev2 Current, Maglev3 Current.
		snprintf(data, 128, "%d, %d, %d, %d",
			motors[0]->amps, motors[1]->amps, motors[2]->amps, motors[3]->amps);
		ethernet_add_data_to_packet(M_CUR, -1, -1, data);

		// Maglev0 Temperature (0, 1, 2, 3), Maglev1 Temperature (0, 1, 2, 3)
		// Maglev2 Temperature (0, 1, 2, 3), Maglev3 Temperature (0, 1, 2, 3).
		snprintf(data, 128, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			motors[0]->temperatures[0], motors[0]->temperatures[1], motors[0]->temperatures[2], motors[0]->temperatures[3],
			motors[1]->temperatures[0], motors[1]->temperatures[1], motors[1]->temperatures[2], motors[1]->temperatures[3],
			motors[2]->temperatures[0], motors[2]->temperatures[1], motors[2]->temperatures[2], motors[2]->temperatures[3],
			motors[3]->temperatures[0], motors[3]->temperatures[1], motors[3]->temperatures[2], motors[3]->temperatures[3]);
		ethernet_add_data_to_packet(M_T, -1, -1, data);

		// Maglev BMS Front Battery0 Voltage (High, Low), Battery1 Voltage (High, Low), Battery2 Voltage (High, Low).
		for(i=0; i<3; i++) {
			v_low[i] = -1;
			v_high[i] = -1;
		}
		for(i=0; i<3; i++) {
			for(j=0; j<6; j++){
				float v = maglev_bmses[0]->cell_voltages[i][j];
				if(v_low[i] == -1 || v < v_low[i]) v_low[i] = v;
				if(v_high[i] == -1 || v > v_high[i]) v_high[i] = v;
			}
		}
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f, %06.2f, %06.2f",
			v_high[0], v_low[0], v_high[1], v_low[1], v_high[2], v_low[2]);
		ethernet_add_data_to_packet(M_BMS_F_VHL, -1, -1, data);

		// Maglev BMS Back Battery0 Voltage (High, Low), Battery1 Voltage (High, Low), Battery2 Voltage (High, Low)
		for(i=0; i<3; i++) {
			v_low[i] = -1;
			v_high[i] = -1;
		}
		for(i=0; i<3; i++) {
			for(j=0; j<6; j++){
				float v = maglev_bmses[1]->cell_voltages[i][j];
				if(v_low[i] == -1 || v < v_low[i]) v_low[i] = v;
				if(v_high[i] == -1 || v > v_high[i]) v_high[i] = v;
			}
		}
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f, %06.2f, %06.2f",
			v_high[0], v_low[0], v_high[1], v_low[1], v_high[2], v_low[2]);
		ethernet_add_data_to_packet(M_BMS_B_VHL, -1, -1, data);

		// Maglev BMS Front Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent), Battery2 Charge (Coulomb, Percent).
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f, %06.2f, %06.2f",
			maglev_bmses[0]->battery_charge_coulomb[0], maglev_bmses[0]->battery_charge_percent[0],
			maglev_bmses[0]->battery_charge_coulomb[1], maglev_bmses[0]->battery_charge_percent[1],
			maglev_bmses[0]->battery_charge_coulomb[2], maglev_bmses[0]->battery_charge_percent[2]);
		ethernet_add_data_to_packet(M_BMS_F_CHG, -1, -1, data);

		// Maglev BMS Back Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent), Battery2 Charge (Coulomb, Percent).
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f, %06.2f, %06.2f",
			maglev_bmses[1]->battery_charge_coulomb[0], maglev_bmses[1]->battery_charge_percent[0],
			maglev_bmses[1]->battery_charge_coulomb[1], maglev_bmses[1]->battery_charge_percent[1],
			maglev_bmses[1]->battery_charge_coulomb[2], maglev_bmses[1]->battery_charge_percent[2]);
		ethernet_add_data_to_packet(M_BMS_B_CHG, -1, -1, data);

		// Maglev BMS Front Battery0 Temperature (High), Battery1 Temperature (High), Battery2 Temperature (High).
		for(i=0; i<3; i++) {
			int t0 = maglev_bmses[0]->temperatures[i][0];
			int t1 = maglev_bmses[0]->temperatures[i][1];
			t_high[i] = (t0 > t1) ? t0 : t1;
		}
		snprintf(data, 128, "%d, %d, %d",
			t_high[0], t_high[1], t_high[2]);
		ethernet_add_data_to_packet(M_BMS_F_TH, -1, -1, data);

		// Maglev BMS Back Battery0 Temperature (High), Battery1 Temperature (High), Battery2 Temperature (High).
		for(i=0; i<3; i++) {
			int t0 = maglev_bmses[1]->temperatures[i][0];
			int t1 = maglev_bmses[1]->temperatures[i][1];
			t_high[i] = (t0 > t1) ? t0 : t1;
		}
		snprintf(data, 128, "%d, %d, %d",
			t_high[0], t_high[1], t_high[2]);
		ethernet_add_data_to_packet(M_BMS_B_TH, -1, -1, data);

		break;

	case LOG_BRAKING:

		// Braking Front Actuator0 Position, Actuator1 Position.
		snprintf(data, 128, "%d, %d",
			braking_boards[0]->position[0], braking_boards[0]->position[1]);
		ethernet_add_data_to_packet(B_F_POS, -1, -1, data);

		// Braking Back Actuator0 Position, Actuator1 Position.
		snprintf(data, 128, "%d, %d",
			braking_boards[1]->position[0], braking_boards[1]->position[1]);
		ethernet_add_data_to_packet(B_B_POS, -1, -1, data);

		// Braking Front Actuator0 Calibration (Disengaged, Ready, Engaged), Actuator1 Calibration (Disengaged, Ready, Engaged).
		snprintf(data, 128, "%d, %d, %d, %d, %d, %d",
			BRAKING_DISENGAGED_POSITIONS[0][0], braking_boards[0]->calibrated_engaged_pos[0] + READY_OFFET_FROM_ENGAGED, braking_boards[0]->calibrated_engaged_pos[0],
			BRAKING_DISENGAGED_POSITIONS[0][1], braking_boards[0]->calibrated_engaged_pos[1] + READY_OFFET_FROM_ENGAGED, braking_boards[0]->calibrated_engaged_pos[1]);
		ethernet_add_data_to_packet(B_F_POS, -1, -1, data);

		// Braking Back Actuator0 Calibration (Disengaged, Ready, Engaged), Actuator1 Calibration (Disengaged, Ready, Engaged).
		snprintf(data, 128, "%d, %d, %d, %d, %d, %d",
			BRAKING_DISENGAGED_POSITIONS[1][0], braking_boards[1]->calibrated_engaged_pos[0] + READY_OFFET_FROM_ENGAGED, braking_boards[1]->calibrated_engaged_pos[0],
			BRAKING_DISENGAGED_POSITIONS[1][1], braking_boards[1]->calibrated_engaged_pos[1] + READY_OFFET_FROM_ENGAGED, braking_boards[1]->calibrated_engaged_pos[1]);
		ethernet_add_data_to_packet(B_B_POS, -1, -1, data);

		// Braking Front Actuator0 Temperature (0, 1), Actuator1 Temperature (0, 1).
		snprintf(data, 128, "%d, %d, %d, %d",
			braking_boards[0]->temperatures[0], braking_boards[0]->temperatures[1], braking_boards[0]->temperatures[2], braking_boards[0]->temperatures[3]);
		ethernet_add_data_to_packet(B_F_T, -1, -1, data);

		// Braking Back Actuator0 Temperature (0, 1), Actuator1 Temperature (0, 1).
		snprintf(data, 128, "%d, %d, %d, %d",
			braking_boards[0]->temperatures[0], braking_boards[0]->temperatures[1], braking_boards[0]->temperatures[2], braking_boards[0]->temperatures[3]);
		ethernet_add_data_to_packet(B_B_T, -1, -1, data);

		// Braking BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low).
		for(i=0; i<2; i++) {
			v_low[i] = -1;
			v_high[i] = -1;
		}
		for(i=0; i<2; i++) {
			for(j=0; j<5; j++){
				float v = bms_18v5->cell_voltages[i][j];
				if(v_low[i] == -1 || v < v_low[i]) v_low[i] = v;
				if(v_high[i] == -1 || v > v_high[i]) v_high[i] = v;
			}
		}
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			v_high[0], v_low[0], v_high[1], v_low[1]);
		ethernet_add_data_to_packet(B_BMS_VHL, -1, -1, data);

		// Braking BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent).
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			bms_18v5->battery_charge_coulomb[0], bms_18v5->battery_charge_percent[0],
			bms_18v5->battery_charge_coulomb[1], bms_18v5->battery_charge_percent[1]);
		ethernet_add_data_to_packet(B_BMS_CHG, -1, -1, data);

		// Braking BMS Battery0 Temperature (High), Battery1 Temperature (High).
		for(i=0; i<2; i++) {
			int t0 = bms_18v5->temperatures[i][0];
			int t1 = bms_18v5->temperatures[i][1];
			t_high[i] = (t0 > t1) ? t0 : t1;
		}
		snprintf(data, 128, "%d, %d",
			t_high[0], t_high[1]);
		ethernet_add_data_to_packet(B_BMS_TH, -1, -1, data);

		break;

	case LOG_SERV_PROP_PAYLOAD:
		// Serv-Prop Actuator State (ENUM: 0-UNKNOWN, 1-LOWERED, 2-LOWERING, 3-RAISING, 4-RAISED), Payload Actuator State (ENUM: 0-UNKNOWN, 1-LOWERED, 2-LOWERING, 3-RAISING, 4-RAISED), Payload Motor State (ENUM: 0-UNKNOWN, 1-STATIONARY, 2-BACKWARDS, 3-FORWARDS).
		snprintf(data, 128, "%d, %d, %d",
			0, 0, 0);
		ethernet_add_data_to_packet(SP_S, -1, -1, data);

		// Serv-Prop Actuator0 Temperature (0, 1), Serv-Prop Actuator1 Temperature (0, 1), Payload Actuator Temperature (0, 1), Payload Motor Temperature (0, 1).
		snprintf(data, 128, "%d, %d, %d, %d, %d, %d, %d, %d",
				service_prop->temperatures[0], service_prop->temperatures[1], service_prop->temperatures[2], service_prop->temperatures[3],
				payload->temperatures[0], payload->temperatures[1], payload->temperatures[2], payload->temperatures[3]);
		ethernet_add_data_to_packet(SP_T, -1, -1, data);

		// Serv-Prop / Payload BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low).
		for(i=2; i<4; i++) {
			v_low[i] = -1;
			v_high[i] = -1;
		}
		for(i=2; i<4; i++) {
			for(j=0; j<5; j++){
				float v = bms_18v5->cell_voltages[i][j];
				if(v_low[i] == -1 || v < v_low[i]) v_low[i] = v;
				if(v_high[i] == -1 || v > v_high[i]) v_high[i] = v;
			}
		}
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			v_high[0], v_low[0], v_high[1], v_low[1]);
		ethernet_add_data_to_packet(SP_BMS_VHL, -1, -1, data);

		// Serv-Prop / Payload BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent).
		snprintf(data, 128, "%06.2f, %06.2f, %06.2f, %06.2f",
			bms_18v5->battery_charge_coulomb[2], bms_18v5->battery_charge_percent[2],
			bms_18v5->battery_charge_coulomb[3], bms_18v5->battery_charge_percent[3]);
		ethernet_add_data_to_packet(SP_BMS_CHG, -1, -1, data);

		// Serv-Prop / Payload BMS Battery0 Temperature (High), Battery1 Temperature (High).
		for(i=2; i<4; i++) {
			int t0 = bms_18v5->temperatures[i][0];
			int t1 = bms_18v5->temperatures[i][1];
			t_high[i] = (t0 > t1) ? t0 : t1;
		}
		snprintf(data, 128, "%d, %d",
			t_high[0], t_high[1]);
		ethernet_add_data_to_packet(SP_BMS_TH, -1, -1, data);

		break;
	}
}

#if 0
void logData(LOG_TYPE log_type, int index)
{
	FRESULT rc;
	(void) rc;	// Unused variable.
	int i, j;

	rc = f_open_log(log_type, index, FA_WRITE);

	// Seek current position in log file.
	rc = f_lseek_(LOG_POSITIONS[log_type][index]);

	char data[16] = "";

	// Time
	//snprintf(data, 16, "%s", sensorData.time);
	//ethernet_add_data_to_packet(TIME, -1, -1, data);
	//rc = f_write_log(log_type, index, data);
	rc = f_write_log(log_type, index, "WIP");

	switch(log_type) {
	case LOG_POSITION:

		// X Position.
		snprintf(data, 16, "%06.2f", sensorData.positionX);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// X Velocity.
		snprintf(data, 16, "%06.2f", sensorData.velocityX);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// X Acceleration.
		snprintf(data, 16, "%06.2f", sensorData.accelX);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Y Position.
		snprintf(data, 16, "%06.2f", sensorData.positionY);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Y Velocity.
		snprintf(data, 16, "%06.2f", sensorData.velocityY);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Y Acceleration.
		snprintf(data, 16, "%06.2f", sensorData.accelY);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Z Position.
		snprintf(data, 16, "%06.2f", sensorData.positionZ);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Z Velocity.
		snprintf(data, 16, "%06.2f", sensorData.velocityZ);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Z Acceleration.
		snprintf(data, 16, "%06.2f", sensorData.accelZ);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Roll.
		snprintf(data, 16, "%06.2f", sensorData.roll);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Pitch.
		snprintf(data, 16, "%06.2f", sensorData.pitch);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Yaw.
		snprintf(data, 16, "%06.2f", sensorData.yaw);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

		// Contact.
		snprintf(data, 16, "%d", sensorData.contact_sensor_pushed);
		//ethernet_add_data_to_packet(???, index, -1, data);
		rc = f_write_log(log_type, index, data);

#if 1
		// Short Ranging
		for(i=0; i<4; i++) {
			snprintf(data, 16, "%06.2f", motors[i]->short_data[0]);
			ethernet_add_data_to_packet(SR, i, -1, data);
			//rc = f_write_log(log_type, index, data);
		}
#endif // 0|1
		// Newline
		rc = f_write_newline(log_type, index);

		break;
	case LOG_HEMS:

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

		for(j=0; j<3; j++){
			// BMS(index) Voltages
			float v_low = -1;
			float v_high = -1;
			for(i=0; i<6; i++){
				float v = maglev_bmses[index]->cell_voltages[j][i];
				if(v_low == -1 || v < v_low) v_low = v;
				if(v_high == -1 || v > v_high) v_high = v;
				snprintf(data, 16, "%06.2f", v);
				ethernet_add_data_to_packet(BMSV, index, (6*j)+i, data);
			}
			// BMS(index) Low/High Voltages
			snprintf(data, 16, "%06.2f", v_low);
			//ethernet_add_data_to_packet(BMSVL, index, -1, data);
			rc = f_write_log(log_type, index, data);
			snprintf(data, 16, "%06.2f", v_high);
			//ethernet_add_data_to_packet(BMSVH, index, -1, data);
			rc = f_write_log(log_type, index, data);

			// BMS(index) Temperatures
			for(i=0; i<2; i++){
				snprintf(data, 16, "%06.2f", maglev_bmses[index]->temperatures[j][i]);
				ethernet_add_data_to_packet(BMST, index, (2*j)+i, data);
				rc = f_write_log(log_type, index, data);
			}
		}

		break;
	case LOG_BRAKING:
		for(i=0; i<2; i++) {
			// PWM.

			// Direction.

			// Position.

			// Current.

			// Temperature. x2
		}

		break;
	default:
		break;
	}

	f_close_();
}
#endif // 0

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
