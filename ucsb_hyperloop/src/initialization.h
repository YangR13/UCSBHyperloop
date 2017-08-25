#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

// State machine variables
#define PROTOTYPE_PRERUN			0  // Spin up each motor to 0.8V in 10-second windows

// Sensors and system actuation toggles
#define PHOTO_ELECTRIC_ACTIVE		0
#define RANGING_SENSORS_ACTIVE		0//(1 && MOTOR_BOARD_I2C_ACTIVE)
#define ACCEL_ACTIVE				0
#define POSITIONING_ACTIVE			(MOTOR_BOARD_I2C_ACTIVE || PHOTO_ELECTRIC_ACTIVE)
#define MOTOR_BOARD_I2C_ACTIVE		1
#define GATHER_DATA_ACTIVE      	(ACCEL_ACTIVE || PHOTO_ELECTRIC_ACTIVE || RANGING_SENSORS_ACTIVE || MOTOR_BOARD_I2C_ACTIVE || CONTACT_SENSOR_ACTIVE || ACTUATORS_ACTIVE || MAGLEV_BMS_ACTIVE || BMS_18V5_ACTIVE || PWR_DST_BMS_ACTIVE)
#define MAGLEV_BMS_ACTIVE       	0 // You should only turn this off if you've manually checked all the maglev batteries.
#define BMS_18V5_ACTIVE             1 // Set equal to ACTUATORS_ACTIVE?
#define PWR_DST_BMS_ACTIVE          1 // You should only turn this off if you're working with a bare board and not the whole pod.
#define I2C_ACTIVE					(MOTOR_BOARD_I2C_ACTIVE || ACCEL_ACTIVE || MAGLEV_BMS_ACTIVE || BMS_18V5_ACTIVE || PWR_DST_BMS_ACTIVE || ACTUATORS_ACTIVE)
#define	CONTACT_SENSOR_ACTIVE		0

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
#define PRINT_SENSOR_DATA_ACTIVE    1

// For testing - controls whether subsystems.c generates fault transition signals to the subsystem state machines.
#define IGNORE_FAULTS               1

// Initialize all the things
void initializeTimers();
void initializeSensorsAndControls();
void initializeCommunications();

#endif
