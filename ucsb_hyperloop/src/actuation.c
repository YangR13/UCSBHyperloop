#include "actuation.h"
#include "braking_state_machine.h"
#include "maglev_state_machine.h"
#include "payload_actuator_sm.h"
#include "service_propulsion_sm.h"
#include "initialization.h"
#include "I2CPERIPHS.h"
#include "sensor_data.h"
#include "actuation.h"

int prototypeRunStartTime = 0;

void performActuation(){
    // Actuate subsystems based on flags from state machine.
#if BRAKING_ACTIVE
	actuate_brakes();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	actuate_maglev();
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	actuate_payload();
#endif
#if SERVICE_PROPULSION_ACTIVE
	actuate_service();
#endif
}

void actuate_brakes(){
	// TODO: Implement this!
	if(Braking_HSM.engage){
	    if (Braking_HSM.feedback){
	        // Engage with feedback
	    }
	    else{
	        // Engage without feedback
	    }
	}
}

void actuate_maglev(){
	// Apply changes if a transition occurred
	if(Maglev_HSM.update) {
		// Set engine behavior
		if(Maglev_HSM.enable_motors) {
			DEBUGOUT("Engines engaged.\n");
			prototypeRunStartTime = getRuntime()/1000;
#if MAGLEV_BMS_ACTIVE
			maglev_bmses[0]->relay_active_low = 0;
			maglev_bmses[1]->relay_active_low = 0;
#endif
			//update and maintain engine throttle
		}
		else {
			DEBUGOUT("Engines disengaged\n.");
			// Set throttle to 0
			int i = 0;
			for(i = 0; i < NUM_HEMS; i++) {
				set_motor_throttle(i, 0);
			}
#if MAGLEV_BMS_ACTIVE
            maglev_bmses[0]->relay_active_low = 1;
            maglev_bmses[1]->relay_active_low = 1;
#endif
		}
		Maglev_HSM.update = 0;
		DEBUGOUT("\n\n");
	}

	// Update engines, even if a transition did not occur
	if(Maglev_HSM.enable_motors) {
		// Update and maintain engine throttle
		int time_sec = getRuntime()/1000;

		if(PROTOTYPE_PRERUN) {  // PRERUN
			if (time_sec < prototypeRunStartTime + 10) {    // Spin up to tenth power.

				DEBUGOUT("ENGINE 3 ON\n");
				motors[0]->throttle_voltage = 0;
				motors[1]->throttle_voltage = 0;
				motors[2]->throttle_voltage = 0.8;
				motors[3]->throttle_voltage = 0;
			}
			else if (time_sec < prototypeRunStartTime + 20) {   // Spin up to tenth power.
				DEBUGOUT("ENGINE 4 ON\n");
				motors[0]->throttle_voltage = 0;
				motors[1]->throttle_voltage = 0;
				motors[2]->throttle_voltage = 0;
				motors[3]->throttle_voltage = 0.8;
			}
			else if (time_sec < prototypeRunStartTime + 30) {   // Spin up to tenth power.
				DEBUGOUT("ENGINE 1 ON\n");
				motors[0]->throttle_voltage = 0.8;
				motors[1]->throttle_voltage = 0;
				motors[2]->throttle_voltage = 0;
				motors[3]->throttle_voltage = 0;
			}
			else if (time_sec < prototypeRunStartTime + 40) {   // Spin up to tenth power.
				DEBUGOUT("ENGINE 2 ON\n");
				motors[0]->throttle_voltage = 0;
				motors[1]->throttle_voltage = 0.8;
				motors[2]->throttle_voltage = 0;
				motors[3]->throttle_voltage = 0;
			}
		}
		else{
			// This part is currently superseded by the throttle signal as set by the web app!
//                int i;
//                for(i = 0; i < NUM_MOTORS; i++) {
//                    set_motor_throttle(i, 4.0);
//                }
		}
	}
	else {
//        	DEBUGOUT("ENGINES OFF\n");
		// Set throttle to 0
		int i;
		for(i = 0; i < 4; i++) {
			set_motor_throttle(i, 0);
		}
#if MAGLEV_BMS_ACTIVE
        maglev_bmses[0]->relay_active_low = 1;
        maglev_bmses[1]->relay_active_low = 1;
#endif
	}
	// TODO: Update HEMS here.
}

void actuate_payload(){
    /* Add in once merged with declaration of payload
    if (!(payload->times[0][1] > 0 || payload->times[1][1] > 0)){
        if (Payload_Actuators_HSM.enable){
            move_time(payload, 0, Payload_Actuators_HSM.direction, PAYLOAD_MOVE_TIME);
        }
    }
    */
}

void actuate_service(){
    /* Add in once merged with declaration of service
    if (!(service->times[0][1] > 0)){
        if (Service_Propulsion_HSM.actuator_enable){
            move_time(payload, 0, Service_Propulsion_HSM.actuator_direction, SERV_MOTOR_ACT_RUN_TIME);
        }
    }

    if (Service_Propulsion_HSM.motor_enable && service->enable[1] == 0){
        service->direction[1] = Service_Propulsion_HSM.direction;
        service->enable[1] = SERV_MOTOR_DUTY;
    }
     */
}


void set_motor_throttle(int motor_num, float voltage){
  // Set the motor's throttle directly, but only if HEMS is enabled
  #if MOTOR_BOARD_I2C_ACTIVE
    if (motor_num < 4){
        if (voltage <= MAX_THROTTLE_VOLTAGE && voltage >= 0){
            motors[motor_num]->throttle_voltage = voltage;
        }
        else{
            DEBUGOUT("Invalid voltage specified in set_motor_target_throttle");
        }
    }
    else{
        DEBUGOUT("Invalid motor number in set_motor_target_throttle!\n");
    }
  #endif
}
