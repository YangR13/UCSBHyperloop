/* UC Santa Barbara Hyperloop Team
 *
 * UCSB Hyperloop Controller
 *
 * Celeste "Bean Bag" Bean
 * Connor "She Sucks ^^^" Buckland
 * Benoit "Balls" Hartl
 * Cameron "Future Billionaire, TV star, etc." McCarthy
 * Connor.js Mulcahey
 *
 */

#include "initialization.h"
#include "time.h"
#include "board.h" 
#include "temp_press.h"
#include "stdlib.h"
#include "accelerometer.h"
#include "stdio.h"
#include "sensor_data.h"
#include "i2c.h"
#include "photo_electric.h"
#include "pwm.h"
#include "ethernet.h"
#include "ranging.h"
#include "braking.h"

 int main(void)
 {

    /* Initialize the board and clock */
    SystemCoreClockUpdate();
    Board_Init();

    if(PWM_ACTIVE){
        Init_PWM(LPC_PWM1);
        Init_Channel(LPC_PWM1, 1);
        Set_Channel_PWM(LPC_PWM1, 1, 0.5);
    }
    if(GATHER_DATA_ACTIVE){
    	gatherSensorDataTimerInit(LPC_TIMER1, TIMER1_IRQn, 1);
    }
    if(SMOOSHED_ONE_ACTIVE){
    	i2cInit(I2C1, SPEED_100KHZ);
    	smooshedOne = temperaturePressureInit(I2C1);
    	collectCalibrationData(I2C1);
    	getPressure(smooshedOne, I2C1);
    	getPressureFlag = 0;
    }
    if(SMOOSHED_TWO_ACTIVE){
    	i2cInit(I2C2, SPEED_100KHZ);
    	smooshedTwo = temperaturePressureInit(I2C2);
    }
    if(PHOTO_ELECTRIC_ACTIVE){
    	photoelectricInit();
    }
    if(ETHERNET_ACTIVE){
    	ethernetInit(PROTO_UDP, 0);
    }
    if(RANGING_SENSORS_ACTIVE){
    	rangingSensorsInit();
    }

    DEBUGOUT(" UCSB Hyperloop Controller Initialized\n");
    DEBUGOUT("_______________________________________\n\n");

    while( 1 )
    {
        if(stripDetectedFlag) {
            stripDetected();
        	DEBUGOUT("Strip %u, of %u in region %u!\n", strip_count, regional_strip_count, strip_region);
        }

        if(collectDataFlag){
            collectData();
            if (sensorData.dataPrintFlag == 2) { // Print every 20/10 = 2 seconds.
				DEBUGOUT( "longRangingJ22 = %f\t", sensorData.longRangingJ22 );
				DEBUGOUT( "longRangingJ25 = %f\t", sensorData.longRangingJ25 );
				DEBUGOUT( "longRangingJ30 = %f\t", sensorData.longRangingJ30 );
				DEBUGOUT( "longRangingJ31 = %f\n", sensorData.longRangingJ31 );
				DEBUGOUT( "shortRangingJ34 = %f\t", sensorData.shortRangingJ34 );
				DEBUGOUT( "shortRangingJ35 = %f\t", sensorData.shortRangingJ35 );
				DEBUGOUT( "shortRangingJ36 = %f\t", sensorData.shortRangingJ36 );
				DEBUGOUT( "shortRangingJ37 = %f\n", sensorData.shortRangingJ37 );
				DEBUGOUT( "temperature1 = %f\n", sensorData.temp1 );
				DEBUGOUT( "temperature2 = %f\n", sensorData.temp2 );
				DEBUGOUT( "pressure1 = %f\n", sensorData.pressure1 );
				DEBUGOUT( "pressure2 = %f\n", sensorData.pressure2 );
				DEBUGOUT( "accelX = %f\t", sensorData.accelX );
				DEBUGOUT( "accelY = %f\t", sensorData.accelY );
				DEBUGOUT( "accelZ = %f\n", sensorData.accelZ );
				DEBUGOUT( "velocityX = %f\t", sensorData.velocityX );
				DEBUGOUT( "velocityY = %f\t", sensorData.velocityY );
				DEBUGOUT( "velocityZ = %f\n", sensorData.velocityZ );
				DEBUGOUT( "positionX = %f\t", sensorData.positionX );
				DEBUGOUT( "positionY = %f\t", sensorData.positionY );
				DEBUGOUT( "positionZ = %f\n", sensorData.positionZ );
				DEBUGOUT( "\n" );
				sensorData.dataPrintFlag = 0;
            }
        }

        if(sendDataFlag){
            sendData();
            DEBUGOUT( "Sending Data!\n" );
        }

        if(recvDataFlag){
            recvData();
        	DEBUGOUT( "Receiving Data!\n" );
        }

        if(emergencyBrakeFlag){
            emergencyBrake();
        	DEBUGOUT( "Emergency brake signal received!\n" );
        }

    }

    return 0;
 }
