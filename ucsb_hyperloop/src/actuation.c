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
	// TODO: Finish implementing this!
	if(Braking_HSM.engage){
	    // Count if there are any faulted boards before beginning actuation
	    int num_rec_faults = 0;
	    int num_unrec_faults = 0;
	    int i;
        for (i = 0; i < 2; i++){
            if (braking_boards[i]->faulted & 0b01){
                num_rec_faults++;
            }
            if (braking_boards[i]->faulted >> 1){
                num_unrec_faults++;
            }
        }

        if (num_rec_faults % 2 == 0 && num_unrec_faults % 2 == 0){
            // Both boards have the same fault status, may as well actuate both.
            // TODO: Actuate both pairs of brakes.
        }
        else if (num_rec_faults % 2 == 1 && num_unrec_faults % 2 == 0){
            // One board has a recoverable fault, just actuate the other
            for (i = 0; i < 2; i++){
                if (!(braking_boards[i]->faulted & 0b01)){
                    // This board does not have the fault
                    // TODO: ACTUATE THIS BOARD braking_boards[i] ONLY
                }
            }
        }
        else if (num_unrec_faults % 2 == 1){
            // One board has an unrecoverable fault. The other may have a recoverable fault or not, but it's still the better choice
            for (i = 0; i < 2; i++){
                if (!(braking_boards[i]->faulted & 0b10)){
                    // This board does not have the fault
                    // TODO: ACTUATE THIS BOARD braking_boards[i] ONLY
                }
            }
        }

        /*
	    if (Braking_HSM.feedback){
	        // Engage with feedback
	    }
	    else{
	        // Engage without feedback
	    }
	    */
	}
	else{
	    // Call a routine to *stop* the brakes here?
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
	// TODO: Implement this stub.
	int i;
	for(i = 0; i < NUM_PAYLOAD_ACTUATORS; i++){
		// TODO: Check this direction for correctness in relation to hardware signal!!!!
		// ACTUATOR DIRECTIONS = Payload_Actuator_HSM.actuator_direction[i];
		// ACTUATOR ENABLES = Payload_Actuator_HSM.actuator_enable[i];
	}
}

void actuate_service(){
	// TODO: Implement this stub.
    // ACTUATOR DIRECTION = Payload_Actuator_HSM.actuator_direction;
    // ACTUATOR ENABLE = Payload_Actuator_HSM.actuator_enable;
	// MOTOR DIRECTION = Payload_Actuator_HSM.motor_direction;
	// MOTOR ENABLE = Payload_Actuator_HSM.motor_enable;
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
