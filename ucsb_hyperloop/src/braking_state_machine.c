#include "braking_state_machine.h"
#include "qpn_port.h"
#include "logging.h"
#include <stdio.h>

// Macro to re-direct the state machine debug test to DEBUGOUT only if SM_DEBUG is 1
#define SM_DEBUG 0
#define BSP_display(msg) if(SM_DEBUG){DEBUGOUT(msg);}

// The global instance of the state machine object
Braking_HSM_t Braking_HSM;

// Forward declaration of all the possible states
static QState Initial(Braking_HSM_t *me);
static QState Tube(Braking_HSM_t *me);
static QState Tube_timingBlocks(Braking_HSM_t *me);
static QState Tube_timingAllows(Braking_HSM_t *me);
static QState Tube_timingAllows_notBraking(Braking_HSM_t *me);
static QState Tube_timingAllows_notBraking_distanceBlocks(Braking_HSM_t *me);
static QState Tube_timingAllows_notBraking_distanceAllows(Braking_HSM_t *me);
static QState Tube_timingAllows_braking(Braking_HSM_t *me);
static QState Tube_noFeedback(Braking_HSM_t *me);
static QState Tube_done(Braking_HSM_t *me);
static QState Test(Braking_HSM_t *me);
static QState Test_idle(Braking_HSM_t *me);
static QState Test_activated(Braking_HSM_t *me);

/*..........................................................................*/
void initializeBrakingStateMachine(void) {
    QHsm_ctor(&Braking_HSM.super, (QStateHandler)&Initial);
    Braking_HSM.stationary_test = 0;
    Braking_HSM.engage = 0;
    Braking_HSM.feedback = 1;
    Braking_HSM.timer_lockout = 1;
    Braking_HSM.distance_lockout = 1;
    Braking_HSM.stopped = 0;
    QHsm_init((QHsm *)&Braking_HSM);
}

/*..........................................................................*/
static QState Initial(Braking_HSM_t *me) {
    BSP_display("Initial-INIT\n");
    return Q_TRAN(&Tube_timingBlocks);
}
/*..........................................................................*/

static QState Tube(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Tube-INIT\n");
    		return Q_TRAN(&Tube_timingBlocks);
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Tube-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_TEST_ENTER: {
        	BSP_display("Tube - BRAKES_TEST_ENTER\n");
        	return Q_TRAN(&Test);
        }
    }
    return Q_SUPER(&QHsm_top);
}
/*..........................................................................*/

static QState Tube_timingBlocks(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Tube_timingBlocks-INIT\n");
    		return Q_HANDLED();
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingBlocks-ENTRY\n");
            Braking_HSM.engage = 0;
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingBlocks-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_TIMER_PERMIT: {
            BSP_display("Tube_timingBlocks-BRAKES_TIMER_PERMIT\n");
            return Q_TRAN(&Tube_timingAllows);
        }
    }
    return Q_SUPER(&Tube);
}
/*..........................................................................*/

static QState Tube_timingAllows(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Tube_timingAllows-INIT\n");
            Braking_HSM.engage = 0;
            Braking_HSM.timer_lockout = 1;
            return Q_TRAN(&Tube_timingAllows_notBraking);
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingAllows-ENTRY\n");
    		return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingAllows-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_ENGAGE: {
            return Q_TRAN(&Tube_timingAllows_braking);
        }
    }
    return Q_SUPER(&Tube);
}
/*..........................................................................*/

static QState Tube_timingAllows_notBraking(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Tube_timingAllows_notBraking-INIT\n");
            Braking_HSM.engage = 0;
    		return Q_TRAN(&Tube_timingAllows_notBraking_distanceBlocks);
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingAllows_notBraking-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingAllows_notBraking-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_TIMER_REQUIRE: {
            return Q_TRAN(&Tube_noFeedback);
        }
    }
    return Q_SUPER(&Tube_timingAllows);
}
/*..........................................................................*/

static QState Tube_timingAllows_notBraking_distanceBlocks(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Tube_timingAllows_notBraking_distanceBlocks-INIT\n");
            Braking_HSM.engage = 0;
    		return Q_HANDLED();
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingAllows_notBraking_distanceBlocks-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingAllows_notBraking_distanceBlocks-EXIT\n");
            Braking_HSM.engage = 0;
            return Q_HANDLED();
        }
        case BRAKES_DISTANCE_PERMIT: {
        	BSP_display("Tube_timingAllows_notBraking_distanceBlocks - BRAKES_DISTANCE_PERMIT\n");
        	return Q_TRAN(&Tube_timingAllows_notBraking_distanceAllows);
        }
    }
    return Q_SUPER(&Tube_timingAllows_notBraking);
}
/*..........................................................................*/

static QState Tube_timingAllows_notBraking_distanceAllows(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
        case Q_INIT_SIG: {
            BSP_display("Tube_timingAllows_notBraking_distanceAllows-INIT\n");
            Braking_HSM.engage = 0;
            Braking_HSM.distance_lockout = 0;
            return Q_HANDLED();
        }
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingAllows_notBraking_distanceAllows-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingAllows_notBraking_distanceAllows-EXIT\n");
            Braking_HSM.engage = 0;
            return Q_HANDLED();
        }
        case BRAKES_DISTANCE_ENGAGE: {
            BSP_display("Tube_timingAllows_notBraking_distanceAllows - BRAKES_DISTANCE_ENGAGE\n");
            return Q_TRAN(&Tube_timingAllows_braking);
        }
    }
    return Q_SUPER(&Tube_timingAllows_notBraking);
}
/*..........................................................................*/

static QState Tube_timingAllows_braking(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
        case Q_INIT_SIG: {
            BSP_display("Tube_timingAllows_braking-INIT\n");
            Braking_HSM.feedback = 1;
            Braking_HSM.engage = 1;
            return Q_HANDLED();
        }
        case Q_ENTRY_SIG: {
            BSP_display("Tube_timingAllows_braking-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_timingAllows_braking-EXIT\n");
            Braking_HSM.engage = 0;
            return Q_HANDLED();
        }
        case BRAKES_DONE: {
            BSP_display("Tube_timingAllows_braking - BRAKES_DONE\n");
            return Q_TRAN(&Tube_done);
        }
    }
    return Q_SUPER(&Tube_timingAllows);
}
/*..........................................................................*/

static QState Tube_noFeedback(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
        case Q_INIT_SIG: {
            BSP_display("Tube_noFeedback-INIT\n");
            Braking_HSM.feedback = 0;
            Braking_HSM.engage = 1;
            return Q_HANDLED();
        }
        case Q_ENTRY_SIG: {
            BSP_display("Tube_noFeedback-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_noFeedback-EXIT\n");
            Braking_HSM.engage = 0;
            return Q_HANDLED();
        }
        case BRAKES_DONE: {
            BSP_display("Tube_timingAllows_braking - BRAKES_DONE\n");
            return Q_TRAN(&Tube_done);
        }
    }
    return Q_SUPER(&Tube_timingAllows);
}
/*..........................................................................*/

static QState Tube_done(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
        case Q_INIT_SIG: {
            BSP_display("Tube_done-INIT\n");
            Braking_HSM.feedback = 0;
            Braking_HSM.engage = 0;
            Braking_HSM.stopped = 1;
            return Q_HANDLED();
        }
        case Q_ENTRY_SIG: {
            BSP_display("Tube_done-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Tube_done-EXIT\n");
            Braking_HSM.engage = 0;
            Braking_HSM.stopped = 0;
            return Q_HANDLED();
        }
        case BRAKES_ENGAGE: {
            BSP_display("Tube_done - BRAKES_ENGAGE\n");
            return Q_TRAN(&Tube_done);
        }
    }
    return Q_SUPER(&Tube);
}
/*..........................................................................*/

static QState Test(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Test-INIT\n");
            return Q_TRAN(&Test_idle);
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Test-ENTRY\n");
    		return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Test-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_TEST_EXIT: {
        	BSP_display("Test - BRAKES_TEST_EXIT\n");
        	return Q_TRAN(&Tube);
        }
    }
    return Q_SUPER(&QHsm_top);
}
/*..........................................................................*/

static QState Test_idle(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Test_idle-INIT\n");
    		return Q_HANDLED();
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Test_idle-ENTRY\n");
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Test_idle-EXIT\n");
            return Q_HANDLED();
        }
        case BRAKES_ENGAGE: {
        	BSP_display("Test_idle - BRAKES_ENGAGE\n");
        	return Q_TRAN(&Test_activated);
        }
    }
    return Q_SUPER(&Test);
}
/*..........................................................................*/

static QState Test_activated(Braking_HSM_t *me) {
    switch (Q_SIG(me)) {
    	case Q_INIT_SIG: {
    		BSP_display("Test_activated-INIT\n");
    		return Q_HANDLED();
    	}
        case Q_ENTRY_SIG: {
            BSP_display("Test_activated-ENTRY\n");
            Braking_HSM.engage = 1;
            Braking_HSM.feedback = 1;
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            BSP_display("Test_activated-EXIT\n");
            Braking_HSM.engage = 0;
            Braking_HSM.feedback = 0;
            return Q_HANDLED();
        }
        case BRAKES_DISENGAGE: {
        	BSP_display("Test_activated - BRAKES_DISENGAGE\n");
        	return Q_TRAN(&Test_idle);
        }
    }
    return Q_SUPER(&Test);
}
/*..........................................................................*/
