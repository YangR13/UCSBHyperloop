#include "initialization.h"
#include "sensor_data.h"
#include "accelerometer.h"
#include "timer.h"
#include "ranging.h"
#include "temp_press.h"
#include "kinematics.h"
#include "photo_electric.h"
#include "timer.h"

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
        //velocity = getInertialVelocity();
        //sensorData.velocityX = velocity.x;
        //sensorData.velocityY = velocity.y;
        //sensorData.velocityZ = velocity.z;
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
		sensorData.pitch = ((z_1 + z_2 - z_0 - z_3) / 2*(d_F + d_B)) - pitch_i;

		// roll
		sensorData.roll = ((z_0 + z_1 - z_2 - z_3) / 2*(d_L + d_R)) - roll_i;

		// COM vertical displacement
		float z_c = (z_0 - (d_L * sensorData.roll) + (d_F * sensorData.pitch)) - z_ci;

		// yaw
		float yaw_i = (y_0i - y_1i) / (d0 + d1);
		sensorData.yaw = ((y_0 - y_1) / (d0 + d1)) - yaw_i;

		// lateral position
		float y_ci = y_0i - (d0 * yaw_i);
		float y_c = (y_0 - (d0 * sensorData.yaw)) - y_ci;
		//DEBUGOUT("Roll: %f Pitch: %f Yaw: n/a\n", roll, pitch);

		sensorData.velocityY = (((y_0+y_1)/2.0) - sensorData.positionY) / (getRuntime()-sensorData.time_prev);
		sensorData.velocityZ = (((z_0+z_1+z_2+z_3)/4.0) - sensorData.positionZ) / (getRuntime()-sensorData.time_prev);

	}

    if (PHOTO_ELECTRIC_ACTIVE){
    	int i;
        /* Handle Photoelectric Strip Detected */
        if(sensorData.photoelectric1InterruptFlag) {
        	sensorData.oldFinalDistanceX = sensorData.finalDistanceX;
        	sensorData.modAvg = (int)(sensorData.finalDistanceX) % 100;
        	for(i=0; i<4; i++) {
        		sensorData.oldActualDistanceX[i] = sensorData.actualDistanceX[i];
        		if (sensorData.modAvg >= 85.0 || sensorData.modAvg <= 15.0) {
        			if(sensorData.wheelTachAlive[i]) {
        				if(((int)sensorData.actualDistanceX[i] % 100) >= 85.0)
        					sensorData.offsetPositionX[i] = 100.0 - ((int)sensorData.wheelTachPositionX[i] % 100);
        				else if (((int)sensorData.actualDistanceX[i] % 100) <= 15.0)
        					sensorData.offsetPositionX[i] = -1*((int)sensorData.wheelTachPositionX[i] % 100);
        				sensorData.actualDistanceX[i] = sensorData.wheelTachPositionX[i] + sensorData.offsetPositionX[i];
        			}
        		}
        	}
        	sensorData.photoelectric1InterruptFlag = 0;
        	float sum = 0.0;

        	//sum actual distance values
        	for(i=0; i<4; i++) {
        		if(sensorData.wheelTachAlive[i]) {
        			sum += sensorData.actualDistanceX[i];
        		}
        	}
        	sensorData.finalDistanceX = sum/(float)sensorData.numAlive; // average

        	sensorData.velocityX = (sensorData.finalDistanceX - sensorData.oldFinalDistanceX) / (getRuntime()-sensorData.time_prev);
    		sensorData.time_prev = getRuntime();

        	sensorData.wheelTachsDistanceInLastHundred = 0.0; //  reset value
        	//determine if any tachs are dead
        	if ((int)(sensorData.finalDistanceX / 100) > sensorData.deadFlag){
        		for(i=0; i<4; i++){
        			sensorData.wheelTachDistanceInLastHundred[i] = sensorData.actualDistanceX[i] - sensorData.oldActualDistanceX[i];
        			sensorData.wheelTachsDistanceInLastHundred += sensorData.wheelTachDistanceInLastHundred[i];
        		}
        		for(i=0; i<4; i++) {
        			sensorData.threeTachAvg = (sensorData.wheelTachsDistanceInLastHundred - sensorData.wheelTachDistanceInLastHundred[i]) / (sensorData.numAlive - 1);
        			if(fabs(sensorData.wheelTachDistanceInLastHundred[i] - sensorData.threeTachAvg) > 20.0 && sensorData.wheelTachAlive[i]){
        				//this tach is dead
            			sensorData.wheelTachAlive[i] = 0;
            			sensorData.numAlive--;
        			}
        		}
        		sensorData.deadFlag++;
        	}

        	DEBUGOUT("Distance: %f feet\n", sensorData.photoelectric);
        }
    }

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

    if(CONTACT_SENSOR_ACTIVE){
    	int contact_sensor_pushed;
    	contact_sensor_pushed = GPIO_Contact_Sensor_Pushed();
    	sensorData.contact_sensor_pushed = contact_sensor_pushed;
    }

    if (POSITIONING_ACTIVE) {
    	int i;
    	float r = 1.0; // radius of wheel

    	// update tachs
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}

    	//gather wheel tach values
    	for(i=0;i<4;i++){
    		sensorData.wheelTachPositionX[i] = motors[i]->tachometer_counter[1] * r * 3.14159265358979323846 / 5.0;	//multiply by 2piR, divide by number of wheel spokes
    	}
    }

    if (BRAKING_ACTIVE){
        int i = 0;
        for (i = 0; i < 2; i++){
            update_actuator_control(braking_boards[i]);
            update_actuator_board(braking_boards[i]);
        }
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

    if (POSITIONING_ACTIVE){
        DEBUGOUT( "pressure1 = %f\n", sensorData.pressure1 );
        DEBUGOUT( "pressure2 = %f\n", sensorData.pressure2 );
        DEBUGOUT( "accelX = %f\t", sensorData.accelX );
        DEBUGOUT( "accelY = %f\t", sensorData.accelY );
        DEBUGOUT( "accelZ = %f\n", sensorData.accelZ );
        DEBUGOUT( "velocityX = %f\t", sensorData.velocityX );
        DEBUGOUT( "velocityY = %f\t", sensorData.velocityY );
        DEBUGOUT( "velocityZ = %f\n", sensorData.velocityZ );
        DEBUGOUT( "positionX = %f\t", sensorData.positionX );
        DEBUGOUT( "positionY = %f\t", sensorData.positionY );
        DEBUGOUT( "positionZ = %f\n", sensorData.positionZ );
        DEBUGOUT( "Roll = %f\t", sensorData.roll );
        DEBUGOUT( "Pitch = %f\t", sensorData.pitch );
        DEBUGOUT( "Yaw = %f\n", sensorData.yaw );
        DEBUGOUT( "\n" );
    }

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
            DEBUGOUT("count[%d]: %f", i, motors[i]->tachometer_counter[1] * 2 * 3.14159265358979323846); // Also multiply by radius later
            sum += motors[i]->tachometer_counter[1];
        }
        avg = sum/4.0; // average tachometer count based on four wheel tachometers
        dist = avg * 2 * 3.14159265358979323846 * r; // average distance traveled based on four wheel tachometers
        for(i=0; i<NUM_HEMS; i++) {
            DEBUGOUT("Motor %d sensors: RPM0=%d \t RPM1=%d \t CURRENT=%d \t TEMP=%d \t %d \t %d \t %d \t SHORT=%f\n", i, motors[i]->rpm[0], motors[i]->rpm[1], motors[i]->amps, motors[i]->temperatures[0], motors[i]->temperatures[1],motors[i]->temperatures[2],motors[i]->temperatures[3], motors[i]->short_data[0]);
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

    if (CONTACT_SENSOR_ACTIVE){
        DEBUGOUT("contact_sensor_pushed: %d\n", sensorData.contact_sensor_pushed);
    }

    if (BRAKING_ACTIVE){
        DEBUGOUT("Braking board 0 sensor data:\n");
        DEBUGOUT("Thermistors: %d | %d | %d | %d\n", braking_boards[0]->temperatures[0], braking_boards[0]->temperatures[1], braking_boards[0]->temperatures[2], braking_boards[0]->temperatures[3]);
        DEBUGOUT("Position: %d | %d \n", braking_boards[0]->position[0], braking_boards[0]->position[1]);
        DEBUGOUT("Current: %d | %d \n", braking_boards[0]->amps[0], braking_boards[0]->amps[1]);
        DEBUGOUT("Bridge fault flag: %d | %d \n", braking_boards[0]->bridge_fault[0], braking_boards[0]->bridge_fault[1]);
        DEBUGOUT("\n\n");
    }
}
