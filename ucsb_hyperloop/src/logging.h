#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>

#include "ethernet.h"
#include "subsystems.h"
#include "sdcard.h"

typedef enum {
	MAX_EVT
} Event;

typedef enum {
	MAX_ERR
} Error;

#define ERR

void logAllData();
void logData(LOG_TYPE log_type, int index);

void initEventLogFile();
void initErrorLogFile();
void logEventString(char* desc);
void logErrorString(char* desc);
void logStateMachineEvent(int sig);
void logEvent(Event evt);
void logError(Error err);

// Helper functions.
void sprintFloatIfActive(char* out, float in, int active);

#endif // LOGGING_H_
