#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include "board.h"
#include "ranging.h"
#include "I2CPERIPHS.h"
#include "actuators.h"

void 	collectCalibrationData(I2C_ID_T id);
void    collectData();
void    printSensorData();
void    gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);

HEMS *motors[NUM_HEMS];
Maglev_BMS *maglev_bmses[NUM_MAGLEV_BMS];
ACTUATORS *braking_boards[2]; // 2 Braking boards.

typedef struct{

  float x;
  float y;
  float z;

} XYZ;

typedef struct{

  uint32_t dataPrintFlag;

  char collectionTime[16];

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

  float pressure1;
  float pressure2;
  float power;

  float photoelectric;

  rangingData shortRangingData;
  rangingData longRangingData;
  int contact_sensor_pushed;


} sensor;
sensor sensorData;

uint8_t getPressureFlag;

int CALIBRATE_FLAG;
float pitch_i;
float roll_i;
float z_ci;

#endif
