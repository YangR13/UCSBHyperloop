#include "initialization.h"
#include "sensor_data.h"
#include "accelerometer.h"
#include "timer.h"
#include "ranging.h"
#include "temp_press.h"
#include "kinematics.h"
#include "photo_electric.h"
#include "timer.h"

uint16_t y = 0;
void collectCalibrationData( I2C_ID_T id ){
	XYZ initialAccel;

	initialAccel = getInitialAccelMatrix(id);
	sensorData.initialAccelX = initialAccel.x;
	sensorData.initialAccelY = initialAccel.y;
	sensorData.initialAccelZ = initialAccel.z;

}

int start_pos = 0;
int stepping = 0;
int stepping_direction = 0;
int step_loop_count = 0;

void TIMER3_IRQHandler(void){
    Chip_TIMER_ClearMatch( LPC_TIMER3, 1 );
    NVIC_ClearPendingIRQ(TIMER3_IRQn);

    if (stepping){
        stepping = 0;
        braking_boards[0]->enable[0] = 0;
        braking_boards[0]->enable[1] = 0;
        update_actuator_board(braking_boards[0]);
//        DEBUGOUT("DONE\n");
        DEBUGOUT("Start position was %d, end position is %d\n\n", start_pos, braking_boards[0]->position[1]);
    }

}

void step(int direction){
    // 1 = forwards, 0 - backwards

    step_loop_count = 0;
    stepping = 1;
    start_pos = braking_boards[0]->position[1];
    stepping_direction = direction;

    braking_boards[0]->direction[0] = direction;
    braking_boards[0]->direction[1] = direction;
    braking_boards[0]->enable[0] = 0.10; // 10%
    braking_boards[0]->enable[1] = 0.10; // 10%
    update_actuator_board(braking_boards[0]);
//    Chip_TIMER_Reset(LPC_TIMER3);
//    Chip_TIMER_Enable(LPC_TIMER3);
//    Reset_Timer_Counter(LPC_TIMER3);
//    NVIC_ClearPendingIRQ(TIMER3_IRQn);
//    NVIC_EnableIRQ(TIMER3_IRQn);
}



void collectData(){

	collectDataFlag = 0;
	sensorData.dataPrintFlag += 1;

	XYZ acceleration1, acceleration2, velocity, position;
	//XYZ acceleration, velocity, position;
	//rangingData shortRangingData, longRangingData;
	positionAttitudeData positionAttitude;
	/*
	if (SMOOSHED_ONE_ACTIVE) {

		if(getPressureFlag){
			sensorData.pressure1 = getPressure(smooshedOne, I2C1);
		}
		else{
			sensorData.temp1 = getTemperature(smooshedOne, I2C1);
		}

		acceleration = getAccelerometerData(I2C1);
		sensorData.accelX = acceleration.x;
		sensorData.accelY = acceleration.y;
		sensorData.accelZ = acceleration.z;

		velocity = getInertialVelocity();
		sensorData.velocityX = velocity.x;
		sensorData.velocityY = velocity.y;
		sensorData.velocityZ = velocity.z;

		position = getInertialPosition();
		sensorData.positionX = position.x;
		sensorData.positionY = position.y;
		sensorData.positionZ = position.z;
	}

	if (SMOOSHED_TWO_ACTIVE) {

		if(getPressureFlag){
			sensorData.pressure2 = getPressure(smooshedOne, I2C2);
		}
		else{
			sensorData.temp2 = getTemperature(smooshedOne, I2C2);
		}

	}
	*/
    if (ACCEL_ACTIVE) {

        acceleration1 = getAccelerometerData(I2C2); // NOTE change back to I2C1
        acceleration2 = getAccelerometerData(I2C2);
        sensorData.accelX = (acceleration1.x + acceleration2.x) / 2.0;
        sensorData.accelY = (acceleration1.y + acceleration2.y) / 2.0;
        sensorData.accelZ = (acceleration1.z + acceleration2.z) / 2.0;
        DEBUGOUT("accel %f, %f, %f \n", acceleration1.x, acceleration1.y, acceleration1.z);
        DEBUGOUT("accel %f, %f, %f\n", acceleration2.x, acceleration2.y, acceleration2.z);
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
		DEBUGOUT("Roll: %f Pitch: %f Yaw: n/a\n", roll, pitch);
	}

    if (PHOTO_ELECTRIC_ACTIVE){
        /* Handle Photoelectric Strip Detected */
        if(stripDetectedFlag) {
//            stripDetected();
        	stripDetectedFlag = 0;
            //DEBUGOUT("Strip %u, of %u in region %u!\n", stripCount, regionalStripCount, stripRegion);
            DEBUGOUT("Distance: %f feet\n", sensorData.photoelectric);
            //sensorData.photoelectric+=100.0; // This is moved INTO the handler so we don't lose strips between printouts.
        }
    }

    if(MOTOR_BOARD_I2C_ACTIVE) {
    	int i;
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}

    	if(y%10 == 0) {
    		// Print sensor data at 1Hz.
    		int i;
    		for(i=0; i<NUM_HEMS; i++) {
    			DEBUGOUT("Motor %d sensors: RPM0=%d, RPM1=%d CURRENT=%d, TEMP=%d,%d,%d,%d, SHORT=%f\n", i, motors[i]->rpm[0], motors[i]->rpm[1], motors[i]->amps, motors[i]->temperatures[0], motors[i]->temperatures[1],motors[i]->temperatures[2],motors[i]->temperatures[3], motors[i]->short_data[0]);
    		}
    		DEBUGOUT("\n");
    	}
    }

    if(MAGLEV_BMS_ACTIVE){
    	int i;
    	for (i = 0; i < NUM_MAGLEV_BMS; i++){
    		update_Maglev_BMS(maglev_bmses[i]);
    	}

    	if(y%10 == 0) {
    		// Print sensor data at 1Hz.
    		int i;
    		for(i=0; i<NUM_MAGLEV_BMS; i++) {
    			DEBUGOUT("BMS %d sensors: \n", i);
    			int j = 0;
    			for (j = 0; j < 3; j++){
    				DEBUGOUT("Batt %d: %f v - cell voltages %f | %f | %f | %f | %f | %f - temperatures %d | %d \n", j, maglev_bmses[i]->battery_voltage[j], maglev_bmses[i]->cell_voltages[j][0], maglev_bmses[i]->cell_voltages[j][1], maglev_bmses[i]->cell_voltages[j][2], maglev_bmses[i]->cell_voltages[j][3], maglev_bmses[i]->cell_voltages[j][4], maglev_bmses[i]->cell_voltages[j][5], maglev_bmses[i]->temperatures[j][0], maglev_bmses[i]->temperatures[j][1]);
    			}
    		}
    		DEBUGOUT("\n");
    	}
    }

    if(CONTACT_SENSOR_ACTIVE){
    	int contact_sensor_pushed;
    	contact_sensor_pushed = GPIO_Contact_Sensor_Pushed();
    	sensorData.contact_sensor_pushed = contact_sensor_pushed;
    	DEBUGOUT("contact_sensor_pushed: %d\n", contact_sensor_pushed);
    }

    if (BRAKING_ACTIVE){
        int i = 0;
        //for (i = 0; i < 2; i++){
        update_actuator_control(braking_boards[0]);

            if (stepping){
                step_loop_count++;
//                if (stepping_direction == 1){
//                    if (braking_boards[0]->position[1] < (start_pos - 10)){
                if (step_loop_count == 400){
                    stepping = 0;
                    braking_boards[0]->enable[0] = 0;
                    braking_boards[0]->enable[1] = 0;
//                    DEBUGOUT("TIGHTEN DONE\n");
                    DEBUGOUT("Start position was %d, end position is %d\n\n", start_pos, braking_boards[0]->position[1]);
                    update_actuator_control(braking_boards[0]);
                    step_loop_count = 0;
                }
//                }
//                else if (stepping_direction == 0){
//                    if (braking_boards[0]->position[1] > (start_pos + 10)){
//                        stepping = 0;
//                        braking_boards[0]->enable[0] = 0;
//                        braking_boards[0]->enable[1] = 0;
//                        DEBUGOUT("LOOSEN DONE\n");
//                        DEBUGOUT("Start position was %d, end position is %d\n\n", start_pos, braking_boards[0]->position[1]);
//                        update_actuator_control(braking_boards[0]);
//                    }
//                }
            }

            if(y%400 == 0) {
//                DEBUGOUT("Braking board 0 sensor data:\n");
//                DEBUGOUT("Thermistors: %d | %d | %d | %d\n", braking_boards[0]->temperatures[0], braking_boards[0]->temperatures[1], braking_boards[0]->temperatures[2], braking_boards[0]->temperatures[3]);
//                DEBUGOUT("Position: %d | %d \n", braking_boards[0]->position[0], braking_boards[0]->position[1]);
                DEBUGOUT("%d\n", braking_boards[0]->position[1]);
//                DEBUGOUT("Current: %d | %d \n", braking_boards[0]->amps[0], braking_boards[0]->amps[1]);
//                DEBUGOUT("Bridge fault flag: %d | %d \n", braking_boards[0]->bridge_fault[0], braking_boards[0]->bridge_fault[1]);
//                DEBUGOUT("\n\n");
            }
            if(y%400 == 0){
                update_actuator_board(braking_boards[i]);
            }

        //}
    }



	getPressureFlag = !getPressureFlag; // Toggling between pressure and temperature register loading.

	// If you get rid of this the timing will not work well. Be careful.
	y++;
	if (y == 64800){
	    // This roll-over value needs to be 65536 - (65536 % [largest interval used above]) for even increments.
	    y = 0;
	}


}

void TIMER1_IRQHandler(void){
	collectDataFlag = 1;
	Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

void gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate){
  	timerInit(timer, timerInterrupt, tickRate);
}
