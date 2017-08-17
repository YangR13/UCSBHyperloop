#ifndef TIME_H_
#define TIME_H_

#include "board.h"

// "Tick timer" Settings - to be used for triggering time-interval-based routines
#define TICK_TIMER_FREQ 100 // Hz
uint32_t tick;

// Multiplier allows frequencies below 1 Hz (i.e. periods > 1 sec)
#define MAX_PERIOD_MULTIPLIER 10 // Max period length (sec)

// Flags for time-interval-based routines
#define COLLECT_DATA_FREQ 10 // Hz
uint8_t collectDataFlag;
#define LOG_DATA_FREQ 1 // Hz
uint8_t logDataFlag;
#define PRINT_SENSOR_DATA_PERIOD 3 // sec
uint8_t printSensorDataFlag;
#define COLLECT_BRAKING_POSITION_FREQ 15 // Hz
uint8_t collectBrakingPositionFlag;

void runtimeTimerInit();
void tickTimerInit();
uint32_t getRuntime();	// In milliseconds.
void    Reset_Timer_Counter(LPC_TIMER_T *pTMR);
void 	timerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);
void    TIMER1_IRQHandler(void);

#endif
