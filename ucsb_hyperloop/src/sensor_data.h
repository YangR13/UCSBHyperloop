#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include "board.h"
#include "I2CPERIPHS.h"
#include "bms.h"
#include "actuators.h"

void 	collectCalibrationData(I2C_ID_T id);
void    collectData();
void    printSensorData();
void    gatherSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);
void	collectBrakingPositionData();

HEMS *motors[NUM_HEMS];
Maglev_BMS *maglev_bmses[NUM_MAGLEV_BMS];
BMS_18V5 *bms_18v5;
PWR_DST_BMS *pwr_dst_bms;

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

  float time_prev;

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

  float wheelTachPositionX[4];
  int wheelTachAlive[4];
  int incrementFlag;				//determines when to check tachometer values
  float oldWheelTachPositionX[4];	//Wheel tach position from last check
  float oldPositionX;
  int positioningError;

  float pressure1;
  float pressure2;

  float photoelectric;

  int contact_sensor_pushed;


} sensor;
sensor sensorData;

uint8_t getPressureFlag;

int CALIBRATE_FLAG;
float pitch_i;
float roll_i;
float z_ci;

#endif
