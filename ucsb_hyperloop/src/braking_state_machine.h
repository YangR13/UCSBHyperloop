#ifndef braking_sm_h
#define braking_sm_h
#include <stdint.h>
#include "qepn.h"

// Structure of data members to be shared between main and the state machine
typedef struct Braking_HSM_data {
    QHsm super;
    uint8_t stationary_test;
    uint8_t engage;
    uint8_t feedback;
    uint8_t timer_lockout;
    uint8_t distance_lockout;
    uint8_t stopped;
} Braking_HSM_t;

// The global instance of the state machine object (with added data members defined above)
extern Braking_HSM_t Braking_HSM;

// Initialize function
void initializeBrakingStateMachine(void);

// Signals that can be sent to the state machine
enum Braking_Signals {
	// CONTROL SIGNALS
    BRAKES_STOP_SIG = Q_USER_SIG,
	BRAKES_ENGAGE,
	BRAKES_DISENGAGE,
	BRAKES_TEST_ENTER,
	BRAKES_TEST_EXIT,
    // SENSOR-ORIGIN SIGNALS
	BRAKES_DISTANCE_PERMIT,
	BRAKES_DISTANCE_ENGAGE,
    BRAKES_TIMER_PERMIT,
    BRAKES_TIMER_REQUIRE,
    BRAKES_DONE,
    BRAKES_MAX_SIG,
};

#endif
