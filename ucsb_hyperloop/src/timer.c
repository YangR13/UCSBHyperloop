#include "timer.h"
#include "sensor_data.h"
#include "ethernet.h"

void runtimeTimerInit() {
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();

	Chip_TIMER_Init(LPC_TIMER0);

	// MAGIC CONSTANT 2067 - This was determined relative to the RTC on 4/24/17
	Chip_TIMER_PrescaleSet(LPC_TIMER0, timerFreq/2067);	// Increment counter every millisecond.

	Chip_TIMER_Enable(LPC_TIMER0);
}

void tickTimerInit(){
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
//  pTMR->PC = 0; // Reset Prescale Counter
}

/* Pass in the timer (E.g. LPC_TIMER0), timer interrupt (E.g. TIMER0_IRQn), and a tickRate (E.g. 2000)*/
/* tickRate is the frequency you desire. */
// Current max rate is 1000Hz, if needed, adjust prescaleSet value to be lower?
void timerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate){

	uint32_t timerFreq;
	Chip_TIMER_Init( timer );

	/* Timer rate is system clock rate */
	timerFreq = Chip_Clock_GetSystemClockRate();
    Chip_TIMER_PrescaleSet(timer, timerFreq/2067);


	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset( timer );
	Chip_TIMER_MatchEnableInt( timer, 1 );

	//Chip_TIMER_SetMatch( timer, 1, ( timerFreq / (tickRate * 2) ) );
	Chip_TIMER_SetMatch( timer, 1, (1000 / tickRate) );
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

    // Clear the interrupt
    Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

