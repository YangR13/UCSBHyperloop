#include "sensor_data.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "time.h"
#include "ranging.h"
#include "temp_press.h"
#include "kinematics.h"

void collectCalibrationData(){
	XYZ initialAccel;

	initialAccel = getInitialAccelMatrix();
	sensorData.initialAccelX = initialAccel.x;
	sensorData.initialAccelY = initialAccel.y;
	sensorData.initialAccelZ = initialAccel.z;

}

void collectData(){
	collectDataFlag = 0;
	sensorData.dataPrintFlag += 1;

	XYZ acceleration, rotation, velocity, position;
	rangingData shortRangingData, longRangingData;
	positionAttitudeData positionAttitude;

//	sensorData.temp1 = getTemperature(I2C1);
//	sensorData.temp2 = getTemperature(I2C2);
//
//	sensorData.pressure = getPressure();

	longRangingData = getLongDistance();
	//  sensorData.longRangingJ25 = longRangingData.sensor0;
	//  sensorData.longRangingJ30 = longRangingData.sensor1;
	//  sensorData.longRangingJ22 = longRangingData.sensor2;
	//  sensorData.longRangingJ31 = longRangingData.sensor3;

	shortRangingData = getShortDistance();
	//  sensorData.shortRangingJ36 = shortRangingData.sensor0;
	//  sensorData.shortRangingJ37 = shortRangingData.sensor1;
	//  sensorData.shortRangingJ34 = shortRangingData.sensor2;
	//  sensorData.shortRangingJ35 = shortRangingData.sensor3;
	positionAttitude = computePositionAttitudeRanging(longRangingData, shortRangingData);
	sensorData.positionY = positionAttitude.y;
	sensorData.positionZ = positionAttitude.z;
	sensorData.roll = positionAttitude.roll;
	sensorData.pitch = positionAttitude.pitch;
	sensorData.yaw = positionAttitude.yaw;

//	acceleration = getAccelerometerData();
//	sensorData.accelX = acceleration.x;
//	sensorData.accelY = acceleration.y;
//	sensorData.accelZ = acceleration.z;
//
//	rotation = getGyroscopeData();
//	sensorData.gyroX = rotation.x;
//	sensorData.gyroY = rotation.y;
//	sensorData.gyroZ = rotation.z;
//
//	velocity = getInertialVelocity();
//	sensorData.velocityX = velocity.x;
//	sensorData.velocityY = velocity.y;
//	sensorData.velocityZ = velocity.z;

//	position = getInertialPosition();
//	sensorData.positionX = position.x;
//	sensorData.positionY = position.y;
//	sensorData.positionZ = position.z;

}

void TIMER1_IRQHandler(void){
  collectDataFlag = 1;
  Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

void gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate){
  	timerInit(timer, timerInterrupt, tickRate);
}
