#include "initialization.h"
#include "temp_press.h"
#include "i2c.h"
#include "photo_electric.h"
#include "ethernet.h"
#include "pwm.h"
#include "sensor_data.h"
#include "sdcard.h"
#include "gpio.h"
#include "I2CPERIPHS.h"
#include "timer.h"

void initializeTimers(){
    runtimeTimerInit(); // Timer 0 - runtime timer
    tickTimerInit();    // Timer 1 - 'tick' timer for periodic tasks
    // Timer 2 - formerly used for 'sendDataFlag', now open
    // Timer 3 - formerly used for photoelectric sensor timing, now open
}

// Initialize all sensor and control systems that are enabled via #-defines in initialization.h!
void initializeSensorsAndControls(){

    if(I2C_ACTIVE){
        i2cInit(I2C0, SPEED_100KHZ);
    	i2cInit(I2C1, SPEED_100KHZ);
    	i2cInit(I2C2, SPEED_100KHZ);
    }
    if(ACCEL_ACTIVE){
    	Init_Accel(I2C1);
    	Init_Accel(I2C2);
    	collectCalibrationData(I2C1);
    	collectCalibrationData(I2C2);
    }
    if(PHOTO_ELECTRIC_ACTIVE){
        photoelectricInit();
        sensorData.photoelectric = 0.0;
    }
    if(RANGING_SENSORS_ACTIVE){
        //rangingSensorsInit();
        //CALIBRATE_FLAG = 0;
    }
    if(GPIO_INT_ACTIVE){
        /* Enable GPIO Interrupts */
        GPIO_Interrupt_Enable();
    }

    if(POSITIONING_ACTIVE) {
    	int i;
    	sensorData.incrementFlag = 0;
    	sensorData.time_prev = getRuntime();
    	sensorData.oldPositionX = 0.0;
    	sensorData.validPosition = 0;
    	for(i=0; i<4; i++) {
        	sensorData.wheelTachPositionX[i] = 0.0;
        	sensorData.wheelTachAlive[i] = 1;
        	sensorData.oldWheelTachPositionX[i] = 0.0;
    	}
    }

    if(MOTOR_BOARD_I2C_ACTIVE) {
    	// Create objects to hold parameters of the HEMS boards
        motors[0] = initialize_HEMS(0);   			// Front Left
        motors[1] = initialize_HEMS(1);            	// Back Left
        motors[2] = initialize_HEMS(2);   			// Back Right
        motors[3] = initialize_HEMS(3);          	// Front Right

    	prototypeRunFlag = 0;
    }

    if (MAGLEV_BMS_ACTIVE){
    	uint8_t i;
    	for (i = 0; i < NUM_MAGLEV_BMS; i++){
    		maglev_bmses[i] = initialize_Maglev_BMS(i);
    	}
    }

    if (BMS_18V5_ACTIVE){
        bms_18v5 = initialize_BMS_18V5(0);
    }

    if (PWR_DST_BMS_ACTIVE){
        pwr_dst_bms = initialize_PWR_DST_BMS(0);
    }

    if (CONTACT_SENSOR_ACTIVE){
    	GPIO_Input_Init(GPIO_CONTACT_SENSOR_PORT, GPIO_CONTACT_SENSOR_PIN);
    }

    if(ACTUATORS_ACTIVE) {
    	Init_PWM(LPC_PWM0);
    	Init_PWM(LPC_PWM1);
    }

    if (BRAKING_ACTIVE){
        int i;
        for (i = 0; i < 2; i++){
            braking_boards[i] = initialize_actuator_board(ACTUATOR_BOARD_BRAKING_MIN + i);
        }
        calibration_step = CALIBRATION_DONE;
    }

    if (PAYLOAD_ACTUATORS_ACTIVE){
        payload = initialize_actuator_board(ACTUATOR_BOARD_PAYLOAD);
    }

    if (SERVICE_PROPULSION_ACTIVE){
		service_prop = initialize_actuator_board(ACTUATOR_BOARD_SERVICE_PROPULSION);
		//servprop_drive_forwards(service_prop);	// DEBUG.
		//move_in_dir(service_prop, 1, OUT, 0.3);
    }
}

void initializeCommunications(){
    if(ETHERNET_ACTIVE){
        ethernetInit(PROTO_TCP, 0);
        //sendSensorDataTimerInit(LPC_TIMER2, TIMER2_IRQn, 4);

        /* Handle any Wiznet Interrupts present at initialization */
        if(wizIntFlag) {
            wizIntFunction();
        }
    }
    if(SDCARD_ACTIVE) {
        sdcardInit();
        init_csv_files();
    }
};
