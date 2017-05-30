#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include "board.h"
#include "ranging.h"
#include "I2CPERIPHS.h"

void 	collectCalibrationData(I2C_ID_T id);
void    collectData();
void    TIMER1_IRQHandler(void);
void    gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);

HEMS *motors[NUM_HEMS];
Maglev_BMS *maglev_bmses[NUM_MAGLEV_BMS];


typedef struct{

  float x;
  float y;
  float z;

} XYZ;

typedef struct{

  uint32_t dataPrintFlag;

  XYZ initialAccel1;
  XYZ initialAccel2;

  XYZ accel1;
  XYZ accel2;

  float accelX;
  float accelY;
  float accelZ;

  float velocityX;
  float velocityY;
  float velocityZ;

  float positionX;
  float positionY;
  float positionZ;

  float roll;
  float pitch;
  float yaw;

  float temp1;
  float temp2;
  float temp3;
  float temp4;

  float tacho1;
  float tacho2;
  float tacho3;
  float tacho4;

  float wheelTach1PositionX;
  float wheelTach2PositionX;
  float wheelTach3PositionX;
  float wheelTach4PositionX;

  float actualDistanceX;					// Increments of 100 ft
  float offsetPositionX;					// Delta away from each increment
  int   photoelectric1InterruptFlag;		// Hit Tape
  float photoelectric1InterruptPosition;	// Value used for miss on distance marker
  float modAvg;								// average mod 100, determines whether wheel tachs in range of [0,15] and [85,100]
  float average;							// this is the position that the tachs read when it hits the tape

  float pressure1;
  float pressure2;
  float power;

  float photoelectric;

  rangingData shortRangingData;
  rangingData longRangingData;
  int contact_sensor_pushed;

} sensor;
sensor sensorData;

uint8_t collectDataFlag;
uint8_t getPressureFlag;

int CALIBRATE_FLAG;
float pitch_i;
float roll_i;
float z_ci;

#endif
