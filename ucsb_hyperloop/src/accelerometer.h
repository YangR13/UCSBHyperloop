#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "sensor_data.h"
													// Range = +/-	2G		4G		8G		16G
#define ACCEL_CTRL_REG4_RANGE_SETTING		0x10	// Hex			0x00	0x10	0x0F	0x1F
#define LSM303ACCEL_MG_LSB					2		// mG per LSB	1		2		4		8

#define ACCEL1_MG_LSB_CALIBRATION			0.9694F
#define ACCEL2_MG_LSB_CALIBRATION			0.9185F

#define ACC_ADDRESS      0x19
#define MAG_ADDRESS      0x1E
#define SENSORS_GRAVITY_STANDARD        (9.80665F)  /**< Earth's gravity in m/s^2 */

#define LSM303_REGISTER_ACCEL_CTRL_REG1_A          0x20
#define LSM303_REGISTER_ACCEL_CTRL_REG2_A          0x21
#define LSM303_REGISTER_ACCEL_CTRL_REG3_A          0x22
#define LSM303_REGISTER_ACCEL_CTRL_REG4_A          0x23
#define LSM303_REGISTER_ACCEL_CTRL_REG5_A          0x24
#define LSM303_REGISTER_ACCEL_CTRL_REG6_A          0x25

#define LSM303_REGISTER_ACCEL_OUT_X_L_A            0x28
#define LSM303_REGISTER_ACCEL_OUT_X_H_A            0x29
#define LSM303_REGISTER_ACCEL_OUT_Y_L_A            0x2A
#define LSM303_REGISTER_ACCEL_OUT_Y_H_A            0x2B
#define LSM303_REGISTER_ACCEL_OUT_Z_L_A            0x2C
#define LSM303_REGISTER_ACCEL_OUT_Z_H_A            0x2D

void Init_Accel( I2C_ID_T id );
XYZ getInitialAccelMatrix( I2C_ID_T id );
XYZ getRawAccelerometerData( I2C_ID_T id );
XYZ getSmoothenedAccelerometerData( I2C_ID_T id );

#endif /* ACCELEROMETER_H_ */
