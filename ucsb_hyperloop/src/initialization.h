#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

// State machine variables
#define PROTOTYPE_PRERUN			0  // Spin up each motor to 0.8V in 10-second windows

// Sensors and system actuation toggles
#define PWM_ACTIVE					0
#define PHOTO_ELECTRIC_ACTIVE		0
#define RANGING_SENSORS_ACTIVE		0
#define MOTOR_BOARD_I2C_ACTIVE		0
#define GATHER_DATA_ACTIVE      	(ACCEL_ACTIVE || PHOTO_ELECTRIC_ACTIVE || RANGING_SENSORS_ACTIVE || ACCEL_ACTIVE || MOTOR_BOARD_I2C_ACTIVE || CONTACT_SENSOR_ACTIVE || ACTUATORS_ACTIVE)
#define MAGLEV_BMS_ACTIVE       	0
#define I2C_ACTIVE					(MOTOR_BOARD_I2C_ACTIVE || ACCEL_ACTIVE || MAGLEV_BMS_ACTIVE || ACTUATORS_ACTIVE)
#define	CONTACT_SENSOR_ACTIVE		0
#define X_POS_VEL_ACTIVE			(PHOTO_ELECTRIC_ACTIVE || MOTOR_BOARD_I2C_ACTIVE)
#define ACCEL_ACTIVE				1

// Actuator-based subsystems
#define BRAKING_ACTIVE				1
#define PAYLOAD_ACTUATORS_ACTIVE	0
#define SERVICE_PROPULSION_ACTIVE	0
#define ACTUATORS_ACTIVE            (BRAKING_ACTIVE || PAYLOAD_ACTUATORS_ACTIVE || SERVICE_PROPULSION_ACTIVE)

// Communications
#define ETHERNET_ACTIVE         	1
#define SDCARD_ACTIVE           	0
#define COMMUNICATION_ACTIVE    	(ETHERNET_ACTIVE || SDCARD_ACTIVE)
#define GPIO_INT_ACTIVE         	(ETHERNET_ACTIVE || PHOTO_ELECTRIC_ACTIVE)
#define PRINT_SENSOR_DATA_ACTIVE    0

// Initialize all the things
void initializeTimers();
void initializeSensorsAndControls();
void initializeCommunications();

#endif
