#include "subsystems.h"
#include "maglev_state_machine.h"
#include "braking_state_machine.h"
#include "payload_actuator_sm.h"
#include "service_propulsion_sm.h"
#include "HEMS.h"
#include "initialization.h"

#define IGNORE_FAULTS 1 	// For testing

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
#if SURFACE_PROPULSION_ACTIVE
	initializeServicePropulsionStateMachine();
#endif
}

void dispatch_signal_from_webapp(int signal){
	// Determine the state machine to issue the control signal to, translate it, and dispatch it

	// Braking
	if (signal >= CS_BRAKES_ENGAGE && signal <= CS_BRAKES_TEST_EXIT){
		int new_signal = (signal - CS_BRAKES_ENGAGE) + BRAKES_ENGAGE;
		Q_SIG((QHsm *)&Braking_HSM) = (QSignal)(new_signal);
		QHsm_dispatch((QHsm *)&Braking_HSM);
	}
	// Mag-lev Motors
	else if (signal >= CS_MAGLEV_ENGAGE && signal <= CS_MAGLEV_DISENGAGE){
		int new_signal = (signal - CS_MAGLEV_ENGAGE) + MAGLEV_ENGAGE;
		Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(new_signal);
		QHsm_dispatch((QHsm *)&Maglev_HSM);
	}
	// Payload Actuators
	else if (signal >= CS_ACTUATORS_RAISE && signal <= CS_ACTUATORS_LOWER){
		int new_signal = (signal - CS_ACTUATORS_RAISE) + PA_ADVANCE;
		Q_SIG((QHsm *)&Payload_Actuator_HSM) = (QSignal)(new_signal);
		QHsm_dispatch((QHsm *)&Payload_Actuator_HSM);
	}
	// Surface Propulsion Motors
	else if (signal >= CS_SURFPROP_ACTUATOR_LOWER && signal <= CS_SURFPROP_DISENGAGE){
		int new_signal = (signal - CS_SURFPROP_ACTUATOR_LOWER) + SP_ADVANCE_SIG;
		Q_SIG((QHsm *)&Service_Propulsion_HSM) = (QSignal)(new_signal);
		QHsm_dispatch((QHsm *)&Service_Propulsion_HSM);
	}
	else{
		// Handle "GO" and "ALL STOP" signals here!
	}

}

void generate_signals_from_sensor_data(){
    // Look at sensor data to determine if a state machine transition signal should be sent.

#if BRAKING_ACTIVE
	braking_service_state_machine();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	maglev_service_state_machine();
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	payload_service_state_machine();
#endif
#if SURFACE_PROPULSION_ACTIVE
	surface_propulsion_service_state_machine();
#endif

#if !IGNORE_FAULTS
	generate_faults_from_sensor_data();
#endif

}

void braking_service_state_machine(){
	// TODO: Implement this stub.
}

void maglev_service_state_machine(){
	if (Maglev_HSM.faulted == 0){
		// Nominal transitions - only evaluated/issued when the system is not faulted
		if (Maglev_HSM.send_spunup && (motors[0]->rpm[1] > 500)){
			// Motors are spun up
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_SPUNUP);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
			Maglev_HSM.send_spunup = 0;
		}
		else if (Maglev_HSM.send_spundown && (motors[0]->rpm[1] < 500)){
			// Motors are spun down
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_SPUNDOWN);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
			Maglev_HSM.send_spundown = 0;
		}

		// Fault transitions
		int fault = maglev_fault_from_sensors();
		if (fault == 1){
			// Recoverable fault
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_FAULT_REC);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
		}
		else if (fault == 2){
			// Unrecoverable fault
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_FAULT_UNREC);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
		}

	}
	else if (Maglev_HSM.faulted == 1){
		// Check if the recoverable fault conditions are all cleared
		int fault = maglev_fault_from_sensors();
		if (fault == 0){
			// Recoverable fault is CLEARED
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_FAULT_REC_CLEAR);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
		}
		else if (fault == 2){
			// Unrecoverable fault
			Q_SIG((QHsm *)&Maglev_HSM) = (QSignal)(MAGLEV_FAULT_UNREC);
			QHsm_dispatch((QHsm *)&Maglev_HSM);
		}
	}
}

void payload_service_state_machine(){
	// TODO: Implement this stub.
}

void surface_propulsion_service_state_machine(){
	// TODO: Implement this stub.
}

void generate_faults_from_sensor_data(){
	// Examine sensor data to determine if a fault transition signal should be issued

#if BRAKING_ACTIVE
	braking_fault_from_sensors();
#endif
#if MOTOR_BOARD_I2C_ACTIVE
	maglev_fault_from_sensors();
#endif
#if PAYLOAD_ACTUATORS_ACTIVE
	payload_fault_from_sensors();
#endif
#if SURFACE_PROPULSION_ACTIVE
	surface_fault_from_sensors();
#endif
}

void braking_fault_from_sensors(){
	// TODO: Implement this stub.
}

int maglev_fault_from_sensors(){
	// Calculates and returns whether the motors have a fault condition
	int i, j;
	// Unrecoverable faults
	/*
	• Motor batteries have too low voltage / state of charge
	• Motor batteries have a significant cell voltage imbalance
	• Feedback or control signals are inoperable on one or more motors
	*/

	// Recoverable faults
	// Find the average RPM of all the motors
	int avg_rpm = 0;
	for (i = 0; i < NUM_MOTORS; i++){
		avg_rpm += motors[i]->rpm[1];
	}
	avg_rpm /= NUM_MOTORS;
	for (i = 0; i < NUM_MOTORS; i++){
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
		for (j = 0; j < NUM_THERMISTORS; j++){
			if (motors[i]->temperatures[j] >= HEMS_MAX_TEMP){
				return 1;
			}
		}
	}
	return 0;
}

void payload_fault_from_sensors(){
	// TODO: Implement this stub.
}

void surface_fault_from_sensors(){
	// TODO: Implement this stub.
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
