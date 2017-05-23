#include "accelerometer.h"
#include "i2c.h"
#include "board.h"
#include "stdio.h"

void Init_Accel( I2C_ID_T id ) {
	uint8_t wBuffer[ 2 ];
	uint8_t wBuffer2[ 2 ];

	wBuffer[ 0 ] = ( LSM303_REGISTER_ACCEL_CTRL_REG1_A ); // Control register initializes all.
	wBuffer[ 1 ] = 0x57;

	wBuffer2[ 0 ] = ( LSM303_REGISTER_ACCEL_CTRL_REG4_A );
	wBuffer2[ 1 ] = ACCEL_CTRL_REG4_RANGE_SETTING;

	Chip_I2C_MasterSend( id, ACC_ADDRESS, wBuffer, 2 );
	Chip_I2C_MasterSend( id, ACC_ADDRESS, wBuffer2, 2);
}

/* Finds the initial values of the accelerometer to calibrate accelerometer values. */
XYZ getInitialAccelMatrix( I2C_ID_T id ){
	uint8_t i;

	XYZ intermediateAccel;
	XYZ initialAccel;
	XYZ sumAccel = {0.0, 0.0, 0.0};

	int num = 25;
	for (i = 0; i < num; i++){
		intermediateAccel = getRawAccelerometerData(id);
		sumAccel.x += intermediateAccel.x;
		sumAccel.y += intermediateAccel.y;
		sumAccel.z += intermediateAccel.z;
	}
	initialAccel.x = sumAccel.x / num;
	initialAccel.y = sumAccel.y / num;
	initialAccel.z = sumAccel.z / num;

	// DEBUGOUT("initialAccel[%d] (G) = <%f %f %f>\n", id, initialAccel.x/SENSORS_GRAVITY_STANDARD, initialAccel.y/SENSORS_GRAVITY_STANDARD, initialAccel.z/SENSORS_GRAVITY_STANDARD);

	float alpha = 0.3;
	float beta = 1 - alpha;

	for (i = 0; i < num; i++){
		intermediateAccel = getRawAccelerometerData(id);
		initialAccel.x = intermediateAccel.x*alpha + initialAccel.x*beta;
		initialAccel.y = intermediateAccel.y*alpha + initialAccel.y*beta;
		initialAccel.z = intermediateAccel.z*alpha + initialAccel.z*beta;
	}
	DEBUGOUT("initialAccel[%d] (G) = <%f %f %f>\n", id, initialAccel.x/SENSORS_GRAVITY_STANDARD, initialAccel.y/SENSORS_GRAVITY_STANDARD, initialAccel.z/SENSORS_GRAVITY_STANDARD);
	return initialAccel;
}

XYZ getRawAccelerometerData( I2C_ID_T id ){
	uint8_t rBuffer[ 6 ];
	int16_t concAcceleration[3];
	XYZ rawAcceleration;

	float MG_LSB_CALIBRATION = (id == I2C1) ? ACCEL1_MG_LSB_CALIBRATION : ACCEL2_MG_LSB_CALIBRATION;
	float CALIBRATED_G_LSB = (float) LSM303ACCEL_MG_LSB * MG_LSB_CALIBRATION * 0.001F;

	Chip_I2C_MasterCmdRead( id, ACC_ADDRESS, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, rBuffer, 6 );

	concAcceleration[0] = (int16_t)(rBuffer[0] | (((uint16_t)rBuffer[1]) << 8)) >> 4;
	concAcceleration[1] = (int16_t)(rBuffer[2] | (((uint16_t)rBuffer[3]) << 8)) >> 4;
	concAcceleration[2] = (int16_t)(rBuffer[4] | (((uint16_t)rBuffer[5]) << 8)) >> 4;

	rawAcceleration.x = ((float)concAcceleration[0]) * CALIBRATED_G_LSB * SENSORS_GRAVITY_STANDARD;
	rawAcceleration.y = ((float)concAcceleration[1]) * CALIBRATED_G_LSB * SENSORS_GRAVITY_STANDARD;
	rawAcceleration.z = ((float)concAcceleration[2]) * CALIBRATED_G_LSB * SENSORS_GRAVITY_STANDARD;

	// DEBUGOUT("rawAcceleration[%d] (G) = <%f, %f, %f>\n", id, rawAcceleration.x/SENSORS_GRAVITY_STANDARD, rawAcceleration.y/SENSORS_GRAVITY_STANDARD, rawAcceleration.z/SENSORS_GRAVITY_STANDARD);

	return rawAcceleration;
}

XYZ getSmoothenedAccelerometerData( I2C_ID_T id ){
	XYZ acceleration;
	XYZ rawAcceleration = getRawAccelerometerData(id);

	float alpha = 0.3;
	float beta = 1 - alpha;

	if (id == I2C1){
		acceleration.x = (rawAcceleration.x - sensorData.initialAccel1.x) * alpha + sensorData.accel1.x * beta;
		acceleration.y = (rawAcceleration.y - sensorData.initialAccel1.y) * alpha + sensorData.accel1.y * beta;
		acceleration.z = (rawAcceleration.z - sensorData.initialAccel1.z) * alpha + sensorData.accel1.z * beta;
	}
	if (id == I2C2){
		acceleration.x = (rawAcceleration.x - sensorData.initialAccel2.x) * alpha + sensorData.accel2.x * beta;
		acceleration.y = (rawAcceleration.y - sensorData.initialAccel2.y) * alpha + sensorData.accel2.y * beta;
		acceleration.z = (rawAcceleration.z - sensorData.initialAccel2.z) * alpha + sensorData.accel2.z * beta;
	}
	return acceleration;
}
