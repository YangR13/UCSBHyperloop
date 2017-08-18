/* UC Santa Barbara Hyperloop Team
 *
 * UCSB Hyperloop Controller
 *
 * Celeste "Not Again" Bean
 * Connor "TCP/IP Expert" Buckland
 * "Big" Ben Hartl
 * Cameron "Deep Fried Board" McCarthy
 * Connor "Funny Guy" Mulcahey
 *
 */

// Standard C/LPC40XX libraries
#include "stdlib.h"
#include "stdio.h"
#include "board.h"
#include "time.h"

// Hyperloop specific libraries
#include "initialization.h"
#include "timer.h"
#include "logging.h"
#include "ethernet.h"
#include "sensor_data.h"
#include "subsystems.h"
#include "actuation.h"
#include "braking_state_machine.h"

int main(void)
{
    /* Initialize the board and clock */
    SystemCoreClockUpdate();
    Board_Init();

    initializeTimers();
    initializeCommunications();
    initializeSensorsAndControls();
    initializeSubsystemStateMachines();

    rangingSensorsCalibrate();

    DEBUGOUT("_______________________________________\n");
    DEBUGOUT("UCSB Hyperloop Controller Initialized\n");
    DEBUGOUT("_______________________________________\n");

    // Main control loop
    while( 1 ) {
    	// 1. Process any received ethernet packets as they arrive
        //  1a. Perform actuation of subsystems (web-app may have issued relevant commands)
        // 2. Perform tasks at frequency (10 Hz) as determined by timer interrupts
        //  2a. Gather data from sensors
        //  2b. Log data to web app, SD card, etc.
        //  2c. Service any high-level user command routines ("go", "stop", etc.)
        //  2d. Send signals to state machine to induce transitions as necessary
        //  2e. Based on flags from state machine, perform actuation of subsystems
	
    	// ** HANDLE ETHERNET PACKETS **
		if(ETHERNET_ACTIVE && wizIntFlag) {
			wizIntFunction();	// See ethernet.c

            // ** DO ACTUATIONS FROM WEB APP COMMANDS **
            performActuation(); // See actuation.c
		}

		// If braking is active, poll feedback signal as fast as possible to determine when set point reached
		if (ACTUATORS_ACTIVE && collectBrakingPositionFlag){
			collectBrakingPositionFlag = 0;
			if(BRAKING_ACTIVE) {
			    update_actuator_control(braking_boards[0]);
			    update_actuator_control(braking_boards[1]);
			}

		    if(SERVICE_PROPULSION_ACTIVE) {
		    	update_actuator_control(service_prop);
		    }
		}

		// ** PERIODIC TASKS **
        if(GATHER_DATA_ACTIVE && collectDataFlag){
            // ** GATHER DATA FROM SENSORS **
            collectDataFlag = 0;
            collectData(); 		// See sensor_data.c

             // ** DATA LOGGING **
            if (COMMUNICATION_ACTIVE && logDataFlag){
                logDataFlag = 0;
                logAllData();          // See logging.c
            }

            // Service high-level command routines ('go', 'all stop', 'emergency stop')
            // ** DETERMINE STATE MACHINE TRANSITIONS**
            generate_signals_from_sensor_data(); // See subsystems.c

            // ** DO ACTUATIONS FROM STATE MACHINE FLAGS **
            performActuation(); // See actuation.c
        }

    }  // End of main control loop

    return 0;
}
