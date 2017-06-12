#include "subsystems.h"
#include "maglev_state_machine.h"
#include "braking_state_machine.h"
#include "payload_actuator_sm.h"
#include "service_propulsion_sm.h"
#include "I2CPERIPHS.h"
#include "initialization.h"
#include "sensor_data.h"
#include "logging.h"

#define ISSUE_SIG(hsm, sig) do {\
	Q_SIG((QHsm *)&hsm) = (QSignal)(sig);\
	QHsm_dispatch((QHsm *)&hsm);\
	} while(0)

#define IGNORE_FAULTS 1 	// For testing

const char *control_signal_names[] = {
		// Special sequences
		"GO\0",
		"ALL_STOP\0",
		"EMERGENCY_POD_STOP\0",
		// Brakes
		"BRAKES_ENGAGE\0",
		"BRAKES_DISENGAGE\0",
		"BRAKES_TEST_ENTER\0",
		"BRAKES_TEST_EXIT\0",
		// Payload Actuators
		"ACTUATORS_RAISE\0",
		"ACTUATORS_LOWER\0",
		// Maglev motors
		"ENGAGE_ENGINES_SIG\0", 		//"MAGLEV_ENGAGE\0",
		"DISENGAGE_ENGINES_SIG\0", 		//"MAGLEV_DISENGAGE\0",
		// Service propulsion
		"SERVPROP_ACTUATOR_LOWER\0",
		"SERVPROP_ACTUATOR_RAISE\0",
		"SERVPROP_ENGAGE_FORWARD\0",
		"SERVPROP_ENGAGE_REVERSE\0",
		"SERVPROP_DISENGAGE\0"
	};


void initializeSubsystemStateMachines(){


#if BRAKING_ACTIVE
	initializeBrakingStateMachine();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	initializeMaglevStateMachine();
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	initializePayloadActuatorStateMachine();
#endif
#if SERVICE_PROPULSION_ACTIVE
	initializeServicePropulsionStateMachine();
#endif
}

void dispatch_signal_from_webapp(int signal){
	// Determine the state machine to issue the control signal to, translate it, and dispatch it

	logStateMachineEvent(signal);

	// Braking
	if (signal >= CS_BRAKES_ENGAGE && signal <= CS_BRAKES_TEST_EXIT){
		int new_signal = (signal - CS_BRAKES_ENGAGE) + BRAKES_ENGAGE;
		ISSUE_SIG(Braking_HSM, new_signal);
	}
	// Mag-lev Motors
	else if (signal >= CS_MAGLEV_ENGAGE && signal <= CS_MAGLEV_DISENGAGE){
		int new_signal = (signal - CS_MAGLEV_ENGAGE) + MAGLEV_ENGAGE;
		ISSUE_SIG(Maglev_HSM, new_signal);
	}
	// Payload Actuators
	else if (signal >= CS_ACTUATORS_RAISE && signal <= CS_ACTUATORS_LOWER){
		int new_signal = (signal - CS_ACTUATORS_RAISE) + PA_ADVANCE;
		ISSUE_SIG(Payload_Actuator_HSM, new_signal);
	}
	// Surface Propulsion Motors
	else if (signal >= CS_SERVPROP_ACTUATOR_LOWER && signal <= CS_SERVPROP_DISENGAGE){
		int new_signal = (signal - CS_SERVPROP_ACTUATOR_LOWER) + SP_ADVANCE_SIG;
		ISSUE_SIG(Service_Propulsion_HSM, new_signal);
	}
	// High-level routine signals
	else if (signal == CS_GO){
		// start go routine
		// Make sure to service it in the main loop too.
	}
	else if (signal == CS_ALL_STOP){
		// Stop all subsystems as quickly as possible.
		// Maglev - issue signal to cut throttle
		// Braking, Payload actuators, service propulsion
		//   issue signal to return to closest idle state (or go into 'recoverable fault'?)
		//   Also, in actuation.c, actually intercept the actuator control signals and turn them off explicitly.
	}
	else if (signal == CS_EMERGENCY_POD_STOP){
		// Stop the pod's forward motion
		// Cut maglev throttle
		// Lower payload actuators, if necessary?
		// Activate brakes
	}
	// Else a bad signal was sent? This shouldn't be possible.

}

void generate_signals_from_sensor_data(){
    // Look at sensor data to determine if a state machine transition signal should be sent.

#if !IGNORE_FAULTS
	generate_faults_from_sensor_data();
#endif

#if BRAKING_ACTIVE
	braking_service_state_machine();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	if (!Maglev_HSM.fault){
		maglev_service_state_machine();
	}
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	if (!Payload_Actuator_HSM.fault){
		payload_service_state_machine();
	}
#endif
#if SERVICE_PROPULSION_ACTIVE
	if (!Service_Propulsion_HSM.fault){
		service_propulsion_service_state_machine();
	}
#endif
#if PWR_DST_BMS_ACTIVE
	if (pwr_dst_bms->alarm){
	    // Handle faults indicated by the electronics power distribution board
	    // TODO: Add routine for shut off of all pod systems?
	}
#endif
}

void braking_service_state_machine(){

	if (Braking_HSM.stationary_test){
		/* TODO
		if (ANY CONDITIONS INDICATING MOVEMENT){
			DEBUGOUT("Exiting braking test mode due to movement!!!\n");
			ISSUE_SIG(Braking_HSM, BRAKES_TEST_EXIT)
		}
		 */
	}

    /*
    // TODO - use data for these. MAKE SURE THEY ONLY ISSUE ONCE!

    if (Braking_HSM.timer_lockout && TIMER PASSES FIRST THRESHOLD){
        // Timing profile window started - allow braking now
        ISSUE_SIG(Braking_HSM, BRAKES_TIMER_PERMIT)
    }
    if (!Braking_HSM.engage && TIMER PASSES SECOND THRESHOLD){
        // Timing profile window ended - force braking now.
        ISSUE_SIG(Braking_HSM, BRAKES_TIMER_REQUIRE);
    {

    if (!Braking_HSM.timer_lockout){
        if (Braking_HSM.distance_lockout && DISTANCE PASSES FIRST THRESHOLD){
            // Distance lockout threshold reached - allow braking requests now
            ISSUE_SIG(Braking_HSM, BRAKES_DISTANCE_PERMIT);
        }
        else if (!Braking_HSM.distance_lockout && DISTANCE PASSES SECOND THRESHOLD){
            // Distance threshold reached - request braking now
            ISSUE_SIG(Braking_HSM, BRAKES_DISTANCE_ENGAGE);
        }

    if (!Braking_HSM.stopped && POD STOPPED){
        // Pod came to a stop - stop braking now
        ISSUE_SIG(Braking_HSM, BRAKES_DONE);
    }
    else if (Braking_HSM.stopped && POD MOVES AGAIN AND SERVICE PROP ISN'T ON){
        // Pod started moving again? Need to brake again because we shouldn't have stopped yet.
        ISSUE_SIG(Braking_HSM, BRAKES_ENGAGE);
    }

	*/
}

void maglev_service_state_machine(){
	if (Maglev_HSM.send_spunup && (motors[0]->rpm[1] > 500)){
		// Motors are spun up
		ISSUE_SIG(Maglev_HSM, MAGLEV_SPUNUP);
		Maglev_HSM.send_spunup = 0;
	}
	else if (Maglev_HSM.send_spundown && (motors[0]->rpm[1] < 500)){
		// Motors are spun down
		Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_SPUNDOWN);
		QHsm_dispatch((QHsm *)&Maglev_HSM);
		Maglev_HSM.send_spundown = 0;
	}

}

void payload_service_state_machine(){
	// TODO: Implement this stub.
	int i;

	// Transition signals due to actuators hitting end stops
	for(i = 0; i < NUM_PAYLOAD_ACTUATORS; i++){
		// TODO: FIX THIS IF DOING BOTH TOGETHER?
		if (Payload_Actuator_HSM.actuator_enable[i]){
			if (Payload_Actuator_HSM.actuator_direction[i]){
				// Forward direction
				// if (HIT END STOP){
				//    ISSUE_SIG(PA_ADVANCE_DONE);
				// }
			}
			else{
				// Backwards direction
				// if (HIT END STOP){
				//    ISSUE_SIG(PA_RETRACT_DONE);
				// }
			}
		}
	}

	// TODO: Add signal due to motors supporting weight or not, here!!!
	/*
	if (MOTORS NEWLY SUPPORTING WEIGHT){
		ISSUE_SIG(Payload_Actuator_HSM, PA_SUPPORT_GAINED);
	}
	else if(MOTORS NEWLY NOT SUPPORTING WEIGHT){
	    ISSUE_SIG(Payload_Actuator_HSM, PA_SUPPORT_LOST);
	}
	*/
}

void service_propulsion_service_state_machine(){
	// Issue transition signals if the service propulsion motor actuator has reached an endstop
	/*
	if (Service_Propulsion_Hsm.actuator_enable){
		if (ADVANCE ENDSTOP REACHED){
			ISSUE_SIG(SP_ADVANCE_DONE);
		}
		else if (RETRACT ENDSTOP REACHED){
			ISSUE_SIG(SP_RETRACT_DONE);
		}
	}
	*/
}

void generate_faults_from_sensor_data(){
	// Examine sensor data to determine if a fault transition signal should be issued
	// This function can't (easily) be condensed because of the different signals that need to be issued to each state machine!
	int fault;

#if BRAKING_ACTIVE
	braking_fault_from_sensors();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	if (Maglev_HSM.fault != 2){
		// If the system is not in an unrecoverable fault condition, check for faults
		fault = maglev_fault_from_sensors();
		if (fault > 1){
			// New unrecoverable fault condition has occurred
			ISSUE_SIG(Maglev_HSM, MAGLEV_FAULT_UNREC);
		}
		else if (Maglev_HSM.fault == 0){
			// System currently not faulted at all
			if (fault == 1){
				// New recoverable fault has occurred
				ISSUE_SIG(Maglev_HSM, MAGLEV_FAULT_REC);
			}
			// Else no fault condition is present
		}
		else if (Maglev_HSM.fault == 1){
			if (fault == 0){
				// Recoverable fault condition is newly cleared
				ISSUE_SIG(Maglev_HSM, MAGLEV_FAULT_REC_CLEAR);
			}
			// Else recoverable fault condition is sustained
		}
	}
	// Else system has already entered an unrecoverable fault condition
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	if (Payload_Actuator_HSM.fault != 2){
		// If the system is not in an unrecoverable fault condition, check for faults
		fault = payload_fault_from_sensors();
		if (fault > 1){
			// New unrecoverable fault condition has occurred
			ISSUE_SIG(Payload_Actuator_HSM, PA_FAULT_UNREC);
		}
		else if (Payload_Actuator_HSM.fault == 0){
			// System currently not faulted at all
			if (fault == 1){
				// New recoverable fault has occurred
				ISSUE_SIG(Payload_Actuator_HSM, PA_FAULT_REC);
			}
			// Else no fault condition is present
		}
		else if (Payload_Actuator_HSM.fault == 1){
			if (fault == 0){
				// Recoverable fault condition is newly cleared
				ISSUE_SIG(Payload_Actuator_HSM, PA_FAULT_REC_CLEAR);
			}
			// Else recoverable fault condition is sustained
		}
	}
	// Else system has already entered an unrecoverable fault condition
#endif
#if SERVICE_PROPULSION_ACTIVE
	if (Service_Propulsion_HSM.fault != 2){
		// If the system is not in an unrecoverable fault condition, check for faults
		fault = service_fault_from_sensors();
		if (fault > 1){
			// New unrecoverable fault condition has occurred
			ISSUE_SIG(Service_Propulsion_HSM, SP_FAULT_UNREC);
		}
		else if (Service_Propulsion_HSM.fault == 0){
			// System currently not faulted at all
			if (fault == 1){
				// New recoverable fault has occurred
				ISSUE_SIG(Service_Propulsion_HSM, SP_FAULT_REC);
			}
			// Else no fault condition is present
		}
		else if (Service_Propulsion_HSM.fault == 1){
			if (fault == 0){
				// Recoverable fault condition is newly cleared
				ISSUE_SIG(Service_Propulsion_HSM, SP_FAULT_REC_CLEAR);
			}
			// Else recoverable fault condition is sustained
		}
	}
	// Else system has already entered an unrecoverable fault condition
#endif
}

void braking_fault_from_sensors(){
	// TODO: Implement this stub.

#if BMS_18V5_ACTIVE
    // Handle faults from the 18V5 BMS
    int i;
    for (i = 0; i < 2; i++){
        if (bms_18v5->alarm[i]){
            // DISABLE BRAKING PAIR i
            braking_boards[i]->faulted |= alarm;
        }
    }
#endif

}

int maglev_fault_from_sensors(){
	// Returns current fault condition of maglev subsystem, if any.
	// 0 = none, 1 = recoverable, 2 = unrecoverable
	int i, j;

	// TODO: Unrecoverable faults (new only)
	// • Motor batteries have too low voltage / state of charge
	// • Motor batteries have a significant cell voltage imbalance
	// • Feedback or control signals are inoperable on one or more motors
	/*
	if (UNRECOVERABLE FAULT){
		return 2;
	}
	*/

	// Recoverable faults (new or existing)
	// Find the average RPM of all the motors
	int avg_rpm = 0;
	for (i = 0; i < NUM_HEMS; i++){
		avg_rpm += motors[i]->rpm[1];
	}
	avg_rpm /= NUM_HEMS;
	for (i = 0; i < NUM_HEMS; i++){
		// Check current readings
		if (motors[i]->amps > HEMS_MAX_CURRENT){
			return 1;
		}

		// Check that no motors have an RPM > 25% different from the others
		// TODO: Check that tolerance
		if (motors[i]->rpm[1] < (0.75 * avg_rpm) || (motors[i]->rpm[1]) > (1.25 * avg_rpm)){
			return 1;
		}

		// Check that no thermistors are above the safe temperature
		for (j = 0; j < 4; j++){
			if (motors[i]->temperatures[j] >= HEMS_MAX_TEMP){
				return 1;
			}
		}
	}

#if MAGLEV_BMS_ACTIVE
	// Handle faults from the Maglev BMSes
    int max_alarm = 0;
    for (i = 0; i < 2; i++){
        if (maglev_bmses[i]->alarm > max_alarm){
            max_alarm = maglev_bmses[i]->alarm;
        }
        if (max_alarm > 0){
            return max_alarm;
        }
    }
#endif
	return 0;
}

int payload_fault_from_sensors(){
	// Returns current fault condition of payload actuator subsystem, if any.
	// 0 = none, 1 = recoverable, 2 = unrecoverable
	// TODO: Implement this stub.
	/*
	if (UNRECOVERABLE FAULT){
		return 2;
	}
	else if (RECOVERABLE FAULT){
		return 1;
	}
	 */

#if BMS_18V5_ACTIVE
    // Handle faults from the 18V5 BMS
    if (bms_18v5->alarm[2]){
        return bms_18v5->alarm[2];
    }
#endif

	return 0;
}

int service_fault_from_sensors(){
	// Returns current fault condition of service propulsion subsystem, if any.
	// 0 = none, 1 = recoverable, 2 = unrecoverable
	// TODO: Implement this stub.
	/*
	if (UNRECOVERABLE FAULT){
		return 2;
	}
	else if (RECOVERABLE FAULT){
		return 1;
	}
	 */

#if BMS_18V5_ACTIVE
    // Handle faults from the 18V5 BMS
    if (bms_18v5->alarm[2]){
        return bms_18v5->alarm[2];
    }
#endif

	return 0;
}


void go_routine(){
	// Do some rad stuff.
	/*
	• Raise payload actuators
	• Ensure flag indicating payload is support by actuators has been set
	• Engage magnetic levitation motors
	• Ensure magnetic levitation motors are operating correctly
	• Lower payload actuators
	• Ensure pod is hovering (magnetic levitation motors are operating and payload linear actuators are lowered)
	• Ensure pod location or test run time is past predetermined braking threshold
	• Engage brakes
	• Ensure pod has come to a stop
	• Disengage brakes
	• Disengage magnetic levitation motors
	*/
}


// This assertion function is required for the state machine. It's called if things go haywire.
void Q_onAssert(char const Q_ROM * const Q_ROM_VAR file, int line) {
    DEBUGOUT("Assertion failed in %s, line %d", file, line);
	//exit(-1);
}
