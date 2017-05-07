#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>

#include "ethernet.h"
#include "subsystems.h"
#include "sdcard.h"

#define TYPE_TXT "txt"
#define TYPE_CSV "csv"

typedef enum {
	LOG_POSITION,
	LOG_HEMS,
	LOG_MAGLEV_BMS,
	NUM_LOGS
} LOG_TYPE;

static const char LOG_TYPE_STRINGS[NUM_LOGS][8] = {
	"pos",
	"hems",
	"ml-bat",
};

static long LOG_POSITIONS[NUM_LOGS][6];

#define FILE_POSITION "position"
#define FILE_HEMS "maglev"
#define FILE_BMS "bms"

typedef enum {
	MAX_EVT
} Event;

typedef enum {
	MAX_ERR
} Error;

#define ERR

char g_log_directory[9];

FRESULT f_open_log(FIL *fp, LOG_TYPE log_type, int index, BYTE mode);
FRESULT f_write_log(FIL *fp, LOG_TYPE log_type, int index, char* data);
FRESULT f_write_newline(FIL *fp, LOG_TYPE log_type, int index);

void get_filepath(char* filepath, char* dir, LOG_TYPE log_type, int index, char* filetype);
void init_csv_files();
void create_csv(char* dir, LOG_TYPE log_type, int index);

void logAllData();
void logData(LOG_TYPE log_type, int index);

void initEventLogFile();
void initErrorLogFile();
void logEventString(char* desc);
void logErrorString(char* desc);
void logStateMachineEvent(int sig);
void logEvent(Event evt);
void logError(Error err);

#endif // LOGGING_H_
