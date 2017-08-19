#include "initialization.h"
#include "sensor_data.h"
#include "accelerometer.h"
#include "timer.h"
#include "ranging.h"
#include "temp_press.h"
#include "kinematics.h"
#include "photo_electric.h"
#include "timer.h"
#include "bms.h"

void CalibrationData( I2C_ID_T id ){
	XYZ initialAccel = getInitialAccelMatrix(id);

	if(id == I2C1) {
		sensorData.initialAccel1 = initialAccel;
	}
	if(id == I2C2) {
		sensorData.initialAccel2 = initialAccel;
	}
}

void collectData(){
	sensorData.dataPrintFlag += 1;

	XYZ acceleration1, acceleration2, velocity, position;
	positionAttitudeData positionAttitude;
    if (ACCEL_ACTIVE) {

        sensorData.accel1 = getSmoothenedAccelerometerData(I2C1);
        sensorData.accel2 = getSmoothenedAccelerometerData(I2C2);
        sensorData.accelX = (sensorData.accel1.x + sensorData.accel2.x) / 2.0;
        sensorData.accelY = (sensorData.accel1.y + sensorData.accel2.y) / 2.0;
        sensorData.accelZ = (sensorData.accel1.z + sensorData.accel2.z) / 2.0;
        //DEBUGOUT("accel1 %f, %f, %f \n", sensorData.accel1.x, sensorData.accel1.y, sensorData.accel1.z);
        //DEBUGOUT("accel2 %f, %f, %f\n", sensorData.accel2.x, sensorData.accel2.y, sensorData.accel2.z);
        DEBUGOUT("accel %f, %f, %f\n", sensorData.accelX, sensorData.accelY, sensorData.accelZ);
        velocity = getInertialVelocity();
        sensorData.velocityX = velocity.x;
        sensorData.velocityY = velocity.y;
        sensorData.velocityZ = velocity.z;
        position = getInertialPosition();
        sensorData.positionX = position.x;
        sensorData.positionY = position.y;
        sensorData.positionZ = position.z;
    }

    if(MOTOR_BOARD_I2C_ACTIVE) {
    	int i;
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}
    }

	if(RANGING_SENSORS_ACTIVE) {

		// update z values to height of shortIR sensors
		float z_0 = motors[0]->short_data[0];
		float z_1 = motors[1]->short_data[0];
		float z_2 = motors[2]->short_data[0];
		float z_3 = motors[3]->short_data[0];

		// y0 and y1
		float y_0 = motors[0]->short_data[1];
		float y_1 = motors[1]->short_data[1];

		// pitch
		sensorData.pitch = ((z_1 + z_2 - z_0 - z_3) / (2*(d_F + d_B))) - pitch_i;

		// roll
		sensorData.roll = ((z_0 + z_1 - z_2 - z_3) / (2*(d_L + d_R))) - roll_i;

		// COM vertical displacement
		sensorData.positionZ = (z_0 - (d_L * sensorData.roll) + (d_F * sensorData.pitch)) - z_ci;

		// yaw
		sensorData.yaw = ((y_0 - y_1) / (I_BEAM_RANGNG_FRONT + I_BEAM_RANGNG_BACK)) - yaw_i;

		// lateral position
		sensorData.positionY = (y_0 - (I_BEAM_RANGNG_FRONT * sensorData.yaw)) - y_ci;
		//DEBUGOUT("Roll: %f Pitch: %f Yaw: n/a\n", roll, pitch);
	}

	if(POSITIONING_ACTIVE){
		int i;
		float wheel_tach_combined_sum = 0.0;
		float wheel_tach_dist_since_last_check[4];
		float total_dist_since_last_check = 0.0;
		int numAlive = 0;								//number of non-faulted tachometers

		// Update average wheel tach postition.
		for(i=0; i<4; i++) {
			if(sensorData.wheelTachAlive[i]){
				numAlive++;
				wheel_tach_combined_sum += sensorData.wheelTachPositionX[i];
			}
		}
		sensorData.validPosition = (numAlive > 2);

		sensorData.positionX = (sensorData.validPosition) ? (wheel_tach_combined_sum / (float) numAlive) : -1;

		if(sensorData.incrementFlag < sensorData.positionX / 25.0){		//Update every 25m
			sensorData.incrementFlag++;

			for(i=0; i<4; i++){
				wheel_tach_dist_since_last_check[i] =sensorData.wheelTachPositionX[i] - sensorData.oldWheelTachPositionX[i];
				total_dist_since_last_check += wheel_tach_dist_since_last_check[i];
			}

			float avg_dist_since_last_check = total_dist_since_last_check / 4.0;
			float largest_percent_difference = 0.0;
			int least_accurate_tach = 0;

			//Collect differences in tach values from the last 25m
			for(i=0; i<4; i++){
				float percent_difference = fabs(wheel_tach_dist_since_last_check[i] - avg_dist_since_last_check) / avg_dist_since_last_check;
				if(percent_difference > largest_percent_difference) {
					largest_percent_difference = percent_difference;
					least_accurate_tach = i;
				}
			}
			if(largest_percent_difference > .18) {	// TODO: Make this a constant in a header file.
				sensorData.wheelTachAlive[least_accurate_tach] = 0;
			}

			// Save wheel tach positions for next 25m check.
			for(i=0; i<4; i++) {
				sensorData.oldWheelTachPositionX[i] = sensorData.wheelTachPositionX[i];
			}
		}
		sensorData.velocityX = (sensorData.validPosition) ? (sensorData.positionX - sensorData.oldPositionX) / (getRuntime()-sensorData.time_prev) : -1;

        sensorData.time_prev = getRuntime();
        sensorData.oldPositionX = sensorData.positionX;
	}

	// Photoelectric distance is updated directly in the interrupt handler

    if(MAGLEV_BMS_ACTIVE){
    	int i;
    	for (i = 0; i < NUM_MAGLEV_BMS; i++){
    		update_Maglev_BMS(maglev_bmses[i]);
    	}
    }

    if (BMS_18V5_ACTIVE){
        update_BMS_18V5(bms_18v5);
    }

    if (PWR_DST_BMS_ACTIVE){
        update_PWR_DST_BMS(pwr_dst_bms);
        if ((pwr_dst_bms->alarm & 0b10) == 0b10){
            DEBUGOUT("\n*****\n");
            DEBUGOUT("Fault condition on one or more Electronics Power Distribution batteries!\n");
            DEBUGOUT("Examine details ASAP and power down pod if necessary! Details: \n");
            int j = 0;
            for (j = 0; j < 2; j++){
                DEBUGOUT("Batt %d: %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, pwr_dst_bms->battery_voltage[j], pwr_dst_bms->cell_voltages[j][0], pwr_dst_bms->cell_voltages[j][1], pwr_dst_bms->cell_voltages[j][2], pwr_dst_bms->cell_voltages[j][3], pwr_dst_bms->cell_voltages[j][4], pwr_dst_bms->temperatures[j][0], pwr_dst_bms->temperatures[j][1]);
            }
            DEBUGOUT("*****\n\n");

        }
    }

    if(CONTACT_SENSOR_ACTIVE){
    	int contact_sensor_pushed;
    	contact_sensor_pushed = GPIO_Contact_Sensor_Pushed();
    	sensorData.contact_sensor_pushed = contact_sensor_pushed;
    }

    if (BRAKING_ACTIVE){
        int i = 0;
//        for (i = 0; i < 2; i++){
            update_actuator_board(braking_boards[i]);
            update_actuator_calibration(braking_boards[i]);
//        }
    }

    if (PRINT_SENSOR_DATA_ACTIVE){
        if (printSensorDataFlag){
            printSensorData();
        }
    }
}

void printSensorData(){
    // Print all data from active sensors to the debug terminal.
    printSensorDataFlag = 0;

    if (PHOTO_ELECTRIC_ACTIVE){
        /* Handle Photoelectric Strip Detected */
        if(stripDetectedFlag) {
            stripDetectedFlag = 0;
            DEBUGOUT("Distance: %f feet\n", sensorData.photoelectric);
        }
    }

    if (ACCEL_ACTIVE){
//        DEBUGOUT("accel %f, %f, %f \n", acceleration1.x, acceleration1.y, acceleration1.z);
//        DEBUGOUT("accel %f, %f, %f\n", acceleration2.x, acceleration2.y, acceleration2.z);
        DEBUGOUT("accel %f, %f, %f\n", sensorData.accelX, sensorData.accelY, sensorData.accelZ);
    }

    if(POSITIONING_ACTIVE) {
    	int i;
    	for(i=0; i<4; i++) {
    		DEBUGOUT("Wheel %d spokes counter: %d | ", i, motors[i]->wheel_tach_spokes_counter);
    		DEBUGOUT("Wheel %d distance traveled: %f feet\n", i, motors[i]->wheel_tach_spokes_counter * 0.575);
    	}
    }

    if (MOTOR_BOARD_I2C_ACTIVE){
#if 1

        float sum = 0.0; // used to sum up four tachometer distances
        float avg; // average of four tachometer differences
        float dist; // distance traveled based on tachometer
        float r = 1.0; // radius of wheel

        // Print sensor data at 1Hz.
        int i;
        for(i=0; i<NUM_HEMS; i++) {
            //DEBUGOUT("count[%d]: %f", i, motors[i]->tachometer_counter[1] * 2 * 3.14159265358979323846); // Also multiply by radius later
            sum += motors[i]->tachometer_counter[1];
        }
        avg = sum/4.0; // average tachometer count based on four wheel tachometers
        dist = avg * 2 * 3.14159265358979323846 * r; // average distance traveled based on four wheel tachometers
        for(i=0; i<NUM_HEMS; i++) {
            DEBUGOUT("Motor %d sensors: RPM0=%d \t RPM1=%d \t DAC=%.2f \t CURRENT=%d \t TEMP=%d \t %d \t %d \t %d \t SHORT=%f\n", i, motors[i]->rpm[0], motors[i]->rpm[1], motors[i]->DAC_diagnostic, motors[i]->amps, motors[i]->temperatures[0], motors[i]->temperatures[1],motors[i]->temperatures[2],motors[i]->temperatures[3], motors[i]->short_data[0]);
        }
        DEBUGOUT("\n");
#endif
    }

    if (MAGLEV_BMS_ACTIVE){
        int i;
        for(i=0; i<NUM_MAGLEV_BMS; i++) {
            DEBUGOUT("BMS %d sensors: \n", i);
            int j = 0;
            for (j = 0; j < 3; j++){
            	DEBUGOUT("Batt %d: (%f\%)\t %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, maglev_bmses[i]->charge_percent[j], maglev_bmses[i]->battery_voltage[j], maglev_bmses[i]->cell_voltages[j][0], maglev_bmses[i]->cell_voltages[j][1], maglev_bmses[i]->cell_voltages[j][2], maglev_bmses[i]->cell_voltages[j][3], maglev_bmses[i]->cell_voltages[j][4], maglev_bmses[i]->cell_voltages[j][5], maglev_bmses[i]->temperatures[j][0], maglev_bmses[i]->temperatures[j][1]);
            	int k = 0;
            	for (k = 0; k < 6; k++){
            		DEBUGOUT("Cell %d: %fC ", k, maglev_bmses[i]->cell_charge_coulomb[j][k]);
            	}
            }
        }
        DEBUGOUT("\n");
    }

    if (BMS_18V5_ACTIVE){
        DEBUGOUT("18V5 BMS sensors: \n");
        int j = 0;
        for (j = 0; j < 4; j++){
        	DEBUGOUT("Batt %d: (%.2f%%)\t %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, bms_18v5->charge_percent[j] * 100, bms_18v5->battery_voltage[j], bms_18v5->cell_voltages[j][0], bms_18v5->cell_voltages[j][1], bms_18v5->cell_voltages[j][2], bms_18v5->cell_voltages[j][3], bms_18v5->cell_voltages[j][4], bms_18v5->temperatures[j][0], bms_18v5->temperatures[j][1]);
        	DEBUGOUT("Current sensor %d: %f \n", j, bms_18v5->amps);
        	int k = 0;
        	for (k = 0; k < 5; k++){
        		DEBUGOUT("Cell: %d %.2fC ", k, bms_18v5->cell_charge_coulomb[j][k]);
        	}
        }
        DEBUGOUT("\n");
    }

    if (PWR_DST_BMS_ACTIVE){
        DEBUGOUT("Power Distribution BMS sensors: \n");
        int j = 0;
        //for (j = 0; j < 2; j++){
            DEBUGOUT("Batt %d: (%.2f%%)\t %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, pwr_dst_bms->charge_percent[j] * 100, pwr_dst_bms->battery_voltage[j], pwr_dst_bms->cell_voltages[j][0], pwr_dst_bms->cell_voltages[j][1], pwr_dst_bms->cell_voltages[j][2], pwr_dst_bms->cell_voltages[j][3], pwr_dst_bms->cell_voltages[j][4], pwr_dst_bms->temperatures[j][0], pwr_dst_bms->temperatures[j][1]);
        //}
        int k = 0;
        for (k = 0; k < 5; k++){
        	DEBUGOUT("Cell: %d %.2fC ", k, pwr_dst_bms->cell_charge_coulomb[j][k]);
        }
        DEBUGOUT("\n");
    }

    if (CONTACT_SENSOR_ACTIVE){
        DEBUGOUT("contact_sensor_pushed: %d\n", sensorData.contact_sensor_pushed);
    }

    if (BRAKING_ACTIVE){
        DEBUGOUT("Braking board 0 sensor data:\n");
        DEBUGOUT("Thermistors: %d | %d | %d | %d\n", braking_boards[0]->temperatures[0], braking_boards[0]->temperatures[1], braking_boards[0]->temperatures[2], braking_boards[0]->temperatures[3]);
        DEBUGOUT("Position: %d | %d \n", braking_boards[0]->position[0], braking_boards[0]->position[1]);
        DEBUGOUT("Current: %d | %d \n", braking_boards[0]->amps[0], braking_boards[0]->amps[1]);
        DEBUGOUT("Bridge fault flag: %d | %d \n", braking_boards[0]->bridge_fault[0], braking_boards[0]->bridge_fault[1]);
        DEBUGOUT("Enable: %d | PWM: %f | Direction: %d | Target_pos: %d\n", braking_boards[0]->enable[0], braking_boards[0]->pwm[0], braking_boards[0]->direction[0], braking_boards[0]->target_pos[0]);
        DEBUGOUT("\n\n");
    }

    if (RANGING_SENSORS_ACTIVE)
    {
		DEBUGOUT("Short-ranging: %f | %f | %f | %f | %f | %f\n",
			motors[0]->short_data[0], motors[1]->short_data[0], motors[2]->short_data[0], motors[3]->short_data[0], motors[0]->short_data[1], motors[1]->short_data[1]);

    	DEBUGOUT("Roll: %f | Pitch: %f | Yaw: %f | PositionX: %f | PositionY: %f | PositionZ: %f\n",
			sensorData.roll*180/PI_CONSTANT, sensorData.pitch*180/PI_CONSTANT, sensorData.yaw*180/PI_CONSTANT, sensorData.positionX, sensorData.positionY, sensorData.positionZ);
    }
}
