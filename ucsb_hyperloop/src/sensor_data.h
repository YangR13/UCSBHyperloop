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

  float initialAccelX;
  float initialAccelY;
  float initialAccelZ;

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

  float pressure1;
  float pressure2;
  float power;

  float photoelectric;

  rangingData shortRangingData;
  rangingData longRangingData;
  int contact_sensor_pushed;


} sensor;

uint8_t getPressureFlag;
sensor sensorData;

int CALIBRATE_FLAG;
float pitch_i;
float roll_i;
float z_ci;

#endif
