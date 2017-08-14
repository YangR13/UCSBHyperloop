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

void collectCalibrationData( I2C_ID_T id ){
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


	if(RANGING_SENSORS_ACTIVE) {


		float d_F = 17.0;
		float d_R = 11.0;
		float d_B = 17.0;
		float d_L = 11.0;

		// update z values to height of shortIR sensors
		float z_0 = motors[0]->short_data[0];
		float z_1 = motors[1]->short_data[0];
		float z_2 = motors[2]->short_data[0];
		float z_3 = motors[3]->short_data[0];

		// initial z values
		float z_0i = motors[0]->short_data[0];
		float z_1i = motors[1]->short_data[0];
		float z_2i = motors[2]->short_data[0];
		float z_3i = motors[3]->short_data[0];

		// d0 and d1 still need to measure these values when we do yaw sensors
		float d0 = 1.0;
		float d1 = 1.0;

		// y0i and y1i
		float y_0i = motors[0]->short_data[1];
		float y_1i = motors[1]->short_data[1];

		// y0 and y1
		float y_0 = motors[0]->short_data[1];
		float y_1 = motors[1]->short_data[1];

		float pitch_i = 0.0;
		float roll_i = 0.0;
		float z_ci = 0.0;
		//initialization calculations
		if(CALIBRATE_FLAG){
			DEBUGOUT("Position Calibrated\n");
			pitch_i = (z_1i + z_2i - z_0i - z_3i) / 2*(d_F + d_B);
			roll_i = (z_0i + z_1i - z_2i - z_3i) / 2*(d_L + d_R);
			z_ci = z_0i - (d_L * roll_i) + (d_F * pitch_i);
		 	CALIBRATE_FLAG = 0;
		}

		// pitch
		float pitch = ((z_1 + z_2 - z_0 - z_3) / 2*(d_F + d_B)) - pitch_i;

		// roll
		float roll = ((z_0 + z_1 - z_2 - z_3) / 2*(d_L + d_R)) - roll_i;

		// COM vertical displacement
		float z_c = (z_0 - (d_L * roll) + (d_F * pitch)) - z_ci;

		// yaw
		float yaw_i = (y_0i - y_1i) / (d0 + d1);
		float yaw = ((y_0 - y_1) / (d0 + d1)) - yaw_i;

		// lateral position
		float y_ci = y_0i - (d0 * yaw_i);
		float y_c = (y_0 - (d0 * yaw)) - y_ci;
		//DEBUGOUT("Roll: %f Pitch: %f Yaw: n/a\n", roll, pitch);
	}

	// Photoelectric distance is updated directly in the interrupt handler

    if(MOTOR_BOARD_I2C_ACTIVE) {
    	int i;
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}
    }

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
            update_actuator_engage(braking_boards[i]);
//        }
    }

    if(SERVICE_PROPULSION_ACTIVE) {
    	update_actuator_board(service_prop);
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

    if (MOTOR_BOARD_I2C_ACTIVE){

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
    }

    if (MAGLEV_BMS_ACTIVE){
        int i;
        for(i=0; i<NUM_MAGLEV_BMS; i++) {
            DEBUGOUT("BMS %d sensors: \n", i);
            int j = 0;
            for (j = 0; j < 3; j++){
                DEBUGOUT("Batt %d: %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, maglev_bmses[i]->battery_voltage[j], maglev_bmses[i]->cell_voltages[j][0], maglev_bmses[i]->cell_voltages[j][1], maglev_bmses[i]->cell_voltages[j][2], maglev_bmses[i]->cell_voltages[j][3], maglev_bmses[i]->cell_voltages[j][4], maglev_bmses[i]->cell_voltages[j][5], maglev_bmses[i]->temperatures[j][0], maglev_bmses[i]->temperatures[j][1]);
            }
        }
        DEBUGOUT("\n");
    }

    if (BMS_18V5_ACTIVE){
        DEBUGOUT("18V5 BMS sensors: \n");
        int j = 0;
        for (j = 0; j < 4; j++){
            DEBUGOUT("Batt %d: %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, bms_18v5->battery_voltage[j], bms_18v5->cell_voltages[j][0], bms_18v5->cell_voltages[j][1], bms_18v5->cell_voltages[j][2], bms_18v5->cell_voltages[j][3], bms_18v5->cell_voltages[j][4], bms_18v5->temperatures[j][0], bms_18v5->temperatures[j][1]);
            DEBUGOUT("Current sensor %d: %f \n", j, bms_18v5->amps);
        }
        DEBUGOUT("\n");
    }

    if (PWR_DST_BMS_ACTIVE){
        DEBUGOUT("Power Distribution BMS sensors: \n");
        int j = 0;
        for (j = 0; j < 2; j++){
            DEBUGOUT("Batt %d: %f v \t cell voltages %f \t %f \t %f \t %f \t %f \t temperatures %d \t %d \n", j, pwr_dst_bms->battery_voltage[j], pwr_dst_bms->cell_voltages[j][0], pwr_dst_bms->cell_voltages[j][1], pwr_dst_bms->cell_voltages[j][2], pwr_dst_bms->cell_voltages[j][3], pwr_dst_bms->cell_voltages[j][4], pwr_dst_bms->temperatures[j][0], pwr_dst_bms->temperatures[j][1]);
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
        DEBUGOUT("Actuator 0 -> Enable: %d | PWM: %f | Direction: %d | Target_pos: %d\n", braking_boards[0]->enable[0], braking_boards[0]->pwm[0], braking_boards[0]->direction[0], braking_boards[0]->target_pos[0]);
        DEBUGOUT("Actuator 1 -> Enable: %d | PWM: %f | Direction: %d | Target_pos: %d\n", braking_boards[0]->enable[1], braking_boards[0]->pwm[1], braking_boards[0]->direction[1], braking_boards[0]->target_pos[1]);
        DEBUGOUT("\n\n");
    }
}
