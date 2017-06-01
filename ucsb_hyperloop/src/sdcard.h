#ifndef __SD_CARD_H
#define __SD_CARD_H

#include <string.h>
#include "board.h"
#include "chip.h"
#include "rtc.h"
#include "initialization.h"

// If this import fails:
// Go into project settings => C/C++ Settings => Settings => MCU C Compiler => Includes
// Then add the path to the 'fatfs/inc' folder, which is located inside the 'src' folder.
#include "ff.h"

/* buffer size (in byte) for R/W operations */
#define SD_BUFFER_SIZE     4096

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

long LOG_POSITIONS[NUM_LOGS][6];

char g_log_directory[9];

void sdcardInit();
void init_csv_files();
void create_csv(char* dir, LOG_TYPE log_type, int index);
FRESULT f_open_log(LOG_TYPE log_type, int index, BYTE mode);
FRESULT f_close_();
FRESULT f_lseek_(DWORD ofs);
FRESULT f_write_log(LOG_TYPE log_type, int index, char* data);
FRESULT f_write_newline(LOG_TYPE log_type, int index);
void get_filepath(char* filepath, char* dir, LOG_TYPE log_type, int index, char* filetype);

#endif	// __SD_CARD_H
