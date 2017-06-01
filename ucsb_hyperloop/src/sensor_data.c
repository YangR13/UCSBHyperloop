#include "initialization.h"
#include "sensor_data.h"
#include "accelerometer.h"
#include "timer.h"
#include "ranging.h"
#include "temp_press.h"
#include "kinematics.h"
#include "photo_electric.h"

int y = 0;
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

	collectDataFlag = 0;
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

    if (PHOTO_ELECTRIC_ACTIVE){
        /* Handle Photoelectric Strip Detected */
        if(sensorData.photoelectric1InterruptFlag) {
//            stripDetected();
        	sensorData.modAvg = (int)(sensorData.average) % 100;
        	if (sensorData.modAvg >= 85.0) {
        		sensorData.offsetPositionX = 100.0 - sensorData.modAvg;
        	    sensorData.actualDistanceX = sensorData.average + sensorData.offsetPositionX;
        	}
        	else if (sensorData.modAvg <= 15.0) {
        	    sensorData.offsetPositionX = sensorData.modAvg;
        	    sensorData.actualDistanceX = sensorData.average - sensorData.offsetPositionX;
        	}
        	sensorData.photoelectric1InterruptFlag = 0;
            //DEBUGOUT("Strip %u, of %u in region %u!\n", stripCount, regionalStripCount, stripRegion);
            DEBUGOUT("Distance: %f feet\n", sensorData.photoelectric);
            //sensorData.photoelectric+=100.0; // This is moved INTO the handler so we don't lose strips between printouts.
        }
    }

    if(MOTOR_BOARD_I2C_ACTIVE) {
    	int i;
    	float sum = 0.0; // used to sum up four tachometer distances
    	float avg; // average of four tachometer differences
    	float dist; // distance traveled based on tachometer
    	float r = 1.0; // radius of wheel
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}

    	if(y%10 == 0) {
    		// Print sensor data at 1Hz.
    		int i;
    		for(i=0; i<NUM_HEMS; i++) {
    			DEBUGOUT("count[%d]: %f", i, motors[i]->tachometer_counter[1] * 2 * 3.14159265358979323846); // Also multiply by radius later
				sum += motors[i]->tachometer_counter[1];
			}
    		avg = sum/4.0; // average tachometer count based on four wheel tachometers
    		dist = avg * 2 * 3.14159265358979323846 * r; // average distance traveled based on four wheel tachometers
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

    if (POSITIONING_ACTIVE) {
    	int i;
    	float minDistAbs, minDistNonAbs; // used to sum up four tachometer distances
    	float r = 1.0; // radius of wheel

    	// update tachs
    	for(i=0; i < NUM_HEMS; i++) {
    		update_HEMS(motors[i]);
    	}

    	//check if any tachs are dead, if they are, set alive value to 0
    	if(motors[0]->rpm[2] == 0) sensorData.wheelTach1Alive = 0;
    	if(motors[1]->rpm[2] == 0) sensorData.wheelTach2Alive = 0;
    	if(motors[2]->rpm[2] == 0) sensorData.wheelTach3Alive = 0;
    	if(motors[3]->rpm[2] == 0) sensorData.wheelTach4Alive = 0;


    	//gather wheel tach values
    	sensorData.wheelTach1PositionX = motors[0]->tachometer_counter[1];
    	sensorData.wheelTach2PositionX = motors[1]->tachometer_counter[1];
    	sensorData.wheelTach3PositionX = motors[2]->tachometer_counter[1];
    	sensorData.wheelTach4PositionX = motors[3]->tachometer_counter[1];

    	//should we put a check here?
    	minDistAbs = fabs(sensorData.wheelTach1PositionX - sensorData.wheelTach2PositionX);
    	sensorData.average = (sensorData.wheelTach1PositionX + sensorData.wheelTach2PositionX) / 2.0;


    	if(sensorData.wheelTach1Alive && sensorData.wheelTach3Alive){
    		if (minDistAbs 			> fabs(sensorData.wheelTach1PositionX - sensorData.wheelTach3PositionX)) {
    			minDistAbs			= fabs(sensorData.wheelTach1PositionX - sensorData.wheelTach3PositionX);
    			sensorData.average	= 	  (sensorData.wheelTach1PositionX + sensorData.wheelTach3PositionX) / 2.0;
    		}
    	}
    	if(sensorData.wheelTach1Alive && sensorData.wheelTach4Alive){
    		if (minDistAbs 			> fabs(sensorData.wheelTach1PositionX - sensorData.wheelTach4PositionX)) {
    			minDistAbs			= fabs(sensorData.wheelTach1PositionX - sensorData.wheelTach4PositionX);
    			sensorData.average	= 	  (sensorData.wheelTach1PositionX + sensorData.wheelTach4PositionX) / 2.0;
    		}
    	}
    	if(sensorData.wheelTach2Alive && sensorData.wheelTach3Alive){
    		if (minDistAbs 			> fabs(sensorData.wheelTach2PositionX - sensorData.wheelTach3PositionX)) {
    			minDistAbs			= fabs(sensorData.wheelTach2PositionX - sensorData.wheelTach3PositionX);
    			sensorData.average	= 	  (sensorData.wheelTach2PositionX + sensorData.wheelTach3PositionX) / 2.0;
    		}
    	}
    	if(sensorData.wheelTach2Alive && sensorData.wheelTach4Alive){
    		if (minDistAbs 			> fabs(sensorData.wheelTach2PositionX - sensorData.wheelTach4PositionX)) {
    			minDistAbs			= fabs(sensorData.wheelTach2PositionX - sensorData.wheelTach4PositionX);
    			sensorData.average	= 	  (sensorData.wheelTach2PositionX + sensorData.wheelTach4PositionX) / 2.0;
    		}
    	}
    	if(sensorData.wheelTach3Alive && sensorData.weelTach4Alive){
    		if (minDistAbs 			> fabs(sensorData.wheelTach3PositionX - sensorData.wheelTach4PositionX)) {
    			minDistAbs			= fabs(sensorData.wheelTach3PositionX - sensorData.wheelTach4PositionX);
    			sensorData.average	= 	  (sensorData.wheelTach3PositionX + sensorData.wheelTach4PositionX) / 2.0;
    		}
    	}


    	sensorData.photoelectric1InterruptPosition = sensorData.average;
    }

	getPressureFlag = !getPressureFlag; // Toggling between pressure and temperature register loading.

	// Currently disabled debug-out printing of data.
#if 0
    if (sensorData.dataPrintFlag == 2) { // Print every 2/1 = 2 seconds.
        DEBUGOUT( "temperature1 = %f\n", sensorData.temp1 );
        DEBUGOUT( "temperature2 = %f\n", sensorData.temp2 );
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
        sensorData.dataPrintFlag = 0;
    }
#endif

}

void TIMER1_IRQHandler(void){
	collectDataFlag = 1;
	Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

void gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate){
  	timerInit(timer, timerInterrupt, tickRate);
}
