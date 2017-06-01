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
    	i2cInit(I2C1, SPEED_100KHZ);
    	i2cInit(I2C2, SPEED_100KHZ);
    }
	if(PWM_ACTIVE){
        Init_PWM(LPC_PWM1);
        Init_Channel(LPC_PWM1, 1);
        Set_Channel_PWM(LPC_PWM1, 1, 0.5);
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
        rangingSensorsInit();
        CALIBRATE_FLAG = 0;
    }
    if(GPIO_INT_ACTIVE){
        /* Enable GPIO Interrupts */
        GPIO_Interrupt_Enable();
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

    if (CONTACT_SENSOR_ACTIVE){
    	GPIO_Input_Init(GPIO_CONTACT_SENSOR_PORT, GPIO_CONTACT_SENSOR_PIN);
    }

    if (BRAKING_ACTIVE){
        i2cInit(I2C1, SPEED_100KHZ);
        Init_PWM(LPC_PWM1);
        int i;
        for (i = 0; i < 2; i++){
            int i = 0;
            braking_boards[i] = initialize_actuator_board(i);
        }
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
