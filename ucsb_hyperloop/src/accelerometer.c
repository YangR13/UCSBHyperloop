#include "accelerometer.h"
#include "i2c.h"
#include "board.h"
#include "stdio.h"

XYZ getAccelerometerData( I2C_ID_T id ){
	uint8_t  		wBuffer[ 2 ];
	uint8_t 		rBuffer[ 6 ];
	int16_t 		concAcceleration[3];
	static XYZ 		rawAcceleration;
	XYZ 		acceleration;

	float alpha = 1;
	//float beta = 1 - alpha;

	float LSM303ACCEL_MG_LSB = (0.0093F);	// 1, 2, 4 or 12 mg per lsb

	wBuffer[ 0 ] = ( LSM303_REGISTER_ACCEL_CTRL_REG1_A ); // Control register initializes all
	wBuffer[ 1 ] = 0x57;

	Chip_I2C_MasterSend( id, ACC_ADDRESS, wBuffer, 2 );
	Chip_I2C_MasterCmdRead( id, ACC_ADDRESS, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, rBuffer, 6 );

	concAcceleration[0] = (int16_t)(rBuffer[0] | (((uint16_t)rBuffer[1]) << 8)) >> 4;
	concAcceleration[1] = (int16_t)(rBuffer[2] | (((uint16_t)rBuffer[3]) << 8)) >> 4;
	concAcceleration[2] = (int16_t)(rBuffer[4] | (((uint16_t)rBuffer[5]) << 8)) >> 4;

	rawAcceleration.x = ((float)concAcceleration[0]) * LSM303ACCEL_MG_LSB; // * SENSORS_GRAVITY_STANDARD;
	rawAcceleration.y = ((float)concAcceleration[1]) * LSM303ACCEL_MG_LSB; // * SENSORS_GRAVITY_STANDARD;
	rawAcceleration.z = ((float)concAcceleration[2]) * LSM303ACCEL_MG_LSB; // * SENSORS_GRAVITY_STANDARD;

	if (id == I2C1){
	  acceleration.x = rawAcceleration.x/*alpha*/ - ACCEL_1_INITIAL_X; //+ sensorData.accelX*beta - sensorData.initialAccelX;
	  acceleration.y = rawAcceleration.y/*alpha*/ - ACCEL_1_INITIAL_Y; //+ sensorData.accelY*beta - sensorData.initialAccelY;
	  acceleration.z = rawAcceleration.z/*alpha*/ - ACCEL_1_INITIAL_Z; //+ sensorData.accelZ*beta - sensorData.initialAccelZ;
	}

	if (id == I2C2){
	  acceleration.x = rawAcceleration.x/*alpha*/ - ACCEL_2_INITIAL_X; //+ sensorData.accelX*beta - sensorData.initialAccelX;
	  acceleration.y = rawAcceleration.y/*alpha*/ - ACCEL_2_INITIAL_Y; //+ sensorData.accelY*beta - sensorData.initialAccelY;
	  acceleration.z = rawAcceleration.z/*alpha*/ - ACCEL_2_INITIAL_Z; //+ sensorData.accelZ*beta - sensorData.initialAccelZ;
	}


	return acceleration;
}

/* Finds the initial values of the accelerometer to calibrate accelerometer values. */
XYZ getInitialAccelMatrix( I2C_ID_T id ){
	uint8_t i;

	XYZ intermediateAccel;
	XYZ initialAccel = {0.0, 0.0, 0.0};

	float alpha = 0.2;
	float beta = 1 - alpha;

	// loop used to avoid potential noise when initializing accelerometers
	for (i = 0; i < 100; i++){
			getAccelerometerData(id);
		}

	for (i = 0; i < 20; i++){
		intermediateAccel = getAccelerometerData(id);
		initialAccel.x = intermediateAccel.x*alpha + initialAccel.x*beta;
		initialAccel.y = intermediateAccel.y*alpha + initialAccel.y*beta;
		initialAccel.z = intermediateAccel.z*alpha + initialAccel.z*beta;
	}

	return initialAccel;
}
