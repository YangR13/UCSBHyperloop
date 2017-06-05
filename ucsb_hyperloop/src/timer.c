#include "timer.h"
#include "sensor_data.h"
#include "ethernet.h"
#include "rtc.h"

void calibrateTimerFrequency() {
	tick = 0;
	calibrationMode = 1;
	calibratedTimerFreq = 0;
	calibrationStartSeconds = rtc_getsec();
	calibrationStartTick = 0;
	calibrationSecondsCount = 0;


	// Use LPC_TIMER1 to count the number of ticks with a prescale of 1.
	uint32_t timerFreq;
	Chip_TIMER_Init( LPC_TIMER1 );

    Chip_TIMER_PrescaleSet(LPC_TIMER1, 1000);

	// Timer setup for match and interrupt at TICKRATE_HZ
	Chip_TIMER_Reset( LPC_TIMER1 );
	Chip_TIMER_MatchEnableInt( LPC_TIMER1, 1 );

	// Match on every timer increment.
	Chip_TIMER_SetMatch( LPC_TIMER1, 1, 1 );
	Chip_TIMER_ResetOnMatchEnable( LPC_TIMER1, 1 );
	Chip_TIMER_Enable( LPC_TIMER1 );

	// Enable timer interrupt
	NVIC_ClearPendingIRQ( TIMER1_IRQn );
	NVIC_EnableIRQ( TIMER1_IRQn );

	// Busy wait for 5+ seconds.
	while(calibrationSecondsCount < 2) {
		__WFI();
	}
	// Stop timer.
	Chip_TIMER_DeInit( LPC_TIMER1 );

	// Timer rate is system clock rate
	timerFreq = Chip_Clock_GetSystemClockRate();
	DEBUGOUT("LPC peripheral clock frequency: %d\n", timerFreq / 4);
	DEBUGOUT("Calibrated timer frequency: %d\n", calibratedTimerFreq);
	calibrationMode = 0;
}

void runtimeTimerInit() {
	Chip_TIMER_Init(LPC_TIMER0);
	Chip_TIMER_PrescaleSet(LPC_TIMER0, calibratedTimerFreq / 1000);	// Increment counter every millisecond.
	Chip_TIMER_Enable(LPC_TIMER0);
}

void tickTimerInit(){
    // Initialize the RTC because we use it for calibration of the tick timer
    rtc_initialize();

    // Initialize tick timer and all time-based routine flags
    timerInit(LPC_TIMER1, TIMER1_IRQn, TICK_TIMER_FREQ);
    tick = 0;
    collectDataFlag = 0;
    logDataFlag = 0;
    printSensorDataFlag = 0;
}

uint32_t getRuntime() {
	return Chip_TIMER_ReadCount(LPC_TIMER0);
}

void Reset_Timer_Counter(LPC_TIMER_T *pTMR) {
  pTMR->TC = 0; // Reset Timer Counter
  pTMR->PC = 0; // Reset Prescale Counter
}

/* Pass in the timer (E.g. LPC_TIMER0), timer interrupt (E.g. TIMER0_IRQn), and a tickRate (E.g. 2000)*/
/* tickRate is the frequency you desire. */
// Current max rate is 1000Hz, if needed, adjust prescaleSet value to be lower?
void timerInit(LPC_TIMER_T * timer, LPC40XX_IRQn_Type timerInterrupt, uint32_t tickRate){
	Chip_TIMER_Init( timer );

	/* Timer rate is system clock rate */
    Chip_TIMER_PrescaleSet(timer, calibratedTimerFreq / tickRate);
//    Chip_TIMER_PrescaleSet(LPC_TIMER1, 1);


	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset( timer );
	Chip_TIMER_MatchEnableInt( timer, 1 );

	//Chip_TIMER_SetMatch( timer, 1, ( timerFreq / (tickRate * 2) ) );
	Chip_TIMER_SetMatch( timer, 1, 1 );
//    Chip_TIMER_SetMatch( timer, 1, (calibratedTimerFreq / tickRate));

	Chip_TIMER_ResetOnMatchEnable( timer, 1 );
	Chip_TIMER_Enable( timer );

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ( timerInterrupt );
	NVIC_EnableIRQ( timerInterrupt );

	return;
}

void TIMER1_IRQHandler(void){
    // Increment the 'ticks' variable
    // If 'ticks' reaches defined thresholds, set the flag to perform tasks whose periods have been reached.

    tick++;

    if(calibrationMode == 1) {
    	// Get RTC seconds.
    	uint32_t secondsRtc = rtc_getsec();
    	if(secondsRtc - calibrationStartSeconds > calibrationSecondsCount) {
    		calibrationSecondsCount++;
    		if(calibrationSecondsCount == 1 && calibrationStartTick == 0) {
    			// Record ticks.
    			calibrationStartTick = tick;
    		}
    		if(calibrationSecondsCount >= 2 && calibratedTimerFreq == 0) {
    			uint32_t tickDiff = tick - calibrationStartTick;
    			// Calculate prescale.
    			calibratedTimerFreq = tickDiff * 1000;
    			DEBUGOUT("Timer calibration complete!\n");
    		    Chip_TIMER_DeInit( LPC_TIMER1 );
    		    NVIC_DisableIRQ(TIMER1_IRQn);
    		}
    	}
    }
    else {
        if (tick % (TICK_TIMER_FREQ / COLLECT_DATA_FREQ) == 0){
            collectDataFlag = 1;
        }
        if (tick % (TICK_TIMER_FREQ / LOG_DATA_FREQ) == 0){
            logDataFlag = 1;
        }
        if (tick % (TICK_TIMER_FREQ * PRINT_SENSOR_DATA_PERIOD) == 0){
            printSensorDataFlag = 1;
        }

        if (tick >= (TICK_TIMER_FREQ * MAX_PERIOD_MULTIPLIER)){
            // Reset ticks to 0 to avoid any overflows (shouldn't be an issue with 32 bit variable anyways)
            tick = 0;
        }
    }

    // Clear the interrupt
    Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

