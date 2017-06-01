#include "sdcard.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

STATIC FATFS fatFS;	/* File system object */
STATIC FIL fileObj;	/* File object */
//STATIC INT buffer[SD_BUFFER_SIZE / 4];		/* Working buffer */
STATIC volatile int32_t sdcWaitExit = 0;
STATIC SDMMC_EVENT_T *event;
STATIC volatile Status  eventResult = ERROR;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* SDMMC card info structure */
SDMMC_CARD_T sdCardInfo;
volatile uint32_t timerCntms = 0; /* Free running milli second timer */

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Delay callback for timed SDIF/SDMMC functions */
STATIC void waitMs(uint32_t time)
{
	uint32_t init = timerCntms;

	/* In an RTOS, the thread would sleep allowing other threads to run.
	   For standalone operation, we just spin on a timer */
	while (timerCntms < init + time) {}
}

/**
 * @brief	Sets up the event driven wakeup
 * @param	pEvent : Event information
 * @return	Nothing
 */
STATIC void setupEvWakeup(void *pEvent)
{
#ifdef SDC_DMA_ENABLE
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(DMA_IRQn);
#endif
	event = (SDMMC_EVENT_T *)pEvent;
	sdcWaitExit = 0;
	eventResult = ERROR;
#ifdef SDC_DMA_ENABLE
	NVIC_EnableIRQ(DMA_IRQn);
#endif /*SDC_DMA_ENABLE*/
}

/**
 * @brief	A better wait callback for SDMMC driven by the IRQ flag
 * @return	0 on success, or failure condition (Nonzero)
 */
STATIC uint32_t waitEvIRQDriven(void)
{
	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdcWaitExit == 0) {}
	if (eventResult) {
		return 0;
	}

	return 1;
}

/* Initialize the Timer at 1us */
STATIC void initAppTimer(void)
{
	/* Setup Systick to tick every 1 milliseconds */
	SysTick_Config(SystemCoreClock / 1000);
}

/* Initialize SD/MMC */
STATIC void initAppSDMMC()
{
	memset(&sdCardInfo, 0, sizeof(sdCardInfo));
	sdCardInfo.evsetup_cb = setupEvWakeup;
	sdCardInfo.waitfunc_cb = waitEvIRQDriven;
	sdCardInfo.msdelay_func = waitMs;

	Board_SDC_Init();

	Chip_SDC_Init(LPC_SDC);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	System tick interrupt handler
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	timerCntms++;
}

#ifdef SDC_DMA_ENABLE
/**
 * @brief	GPDMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	eventResult = Chip_GPDMA_Interrupt(LPC_GPDMA, event->DmaChannel);
	sdcWaitExit = 1;
	NVIC_DisableIRQ(DMA_IRQn);
}
#endif /*SDC_DMA_ENABLE*/

/**
 * @brief	SDC interrupt handler sub-routine
 * @return	Nothing
 */
void SDIO_IRQHandler(void)
{
	int32_t Ret;
#ifdef SDC_DMA_ENABLE
	Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,NULL,0);
#else
	if(event->Index < event->Size) {
		if(event->Dir) { /* receive */
			Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,(uint8_t*)event->Buffer,&event->Index);
		}
		else {
			Ret = Chip_SDMMC_IRQHandler(LPC_SDC, (uint8_t*)event->Buffer,&event->Index,NULL,0);
		}
	}
	else {
		Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,NULL,0);
	}
#endif
	if(Ret < 0) {
		eventResult = ERROR;
		sdcWaitExit = 1;
	}
#ifndef SDC_DMA_ENABLE
	else if(Ret == 0) {
		eventResult = SUCCESS;
		sdcWaitExit = 1;
	}
#endif
}

void sdcardInit() {
	if(!SDCARD_ACTIVE) return;
	initAppSDMMC();

	/* Initialize Repetitive Timer */
	initAppTimer();

	/* Enable SD interrupt */
	NVIC_EnableIRQ(SDC_IRQn);

	f_mount(0, &fatFS);		/* Register volume work area (never fails) */
}

void init_csv_files() {
	if(!SDCARD_ACTIVE) return;
	int i, j;

	/* Get local time */
	RTC rtc;
	rtc_gettime(&rtc);

	// Create a new log directory for this session. Directory names limited to 8 characters.
	snprintf(g_log_directory, 8, "%02d-%02d-%02d", rtc.hour, rtc.min, rtc.sec);
	f_mkdir(g_log_directory);

	// Initialize array for keeping track of file positions.
	for(i=0; i<NUM_LOGS; i++) {
		for(j=0; j<6; j++) {
			LOG_POSITIONS[i][j] = 0;
		}
	}

	// Create new files and add headers.
	create_csv(g_log_directory, LOG_POSITION, 0);

	for(i=0; i<4; i++) {
		create_csv(g_log_directory, LOG_HEMS, i);
	}
	for(i=0; i<6; i++) {
		create_csv(g_log_directory, LOG_MAGLEV_BMS, i);
	}
}

void create_csv(char* dir, LOG_TYPE log_type, int index)
{
	if(!SDCARD_ACTIVE) return;
	FRESULT rc = 0;
	UINT bw;
	f_open_log (log_type, index, FA_WRITE | FA_CREATE_ALWAYS);
	// Add headers.
	char header[128] = "";
	switch(log_type) {
	case LOG_POSITION:
		snprintf(header, 128, "Time,X-Pos,X-Vel,X-Accel,Y-Pos,Y-Vel,Y-Accel,Z-Pos,Z-Vel,Z-Accel,Roll,Pitch,Yaw,Contact\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	case LOG_HEMS:
		snprintf(header, 128, "Time,DAC,Current,RPM,Temp 0,Temp 1,Temp 2,Temp 3\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	case LOG_MAGLEV_BMS:
		snprintf(header, 128, "Time,B0 Low,B0 High,B0 Temp 0,B0 Temp 1,B1 Low,B1 High,B1 Temp 0,B1 Temp 1,B2 Low,B2 High,B2 Temp 0,B2 Temp 1\r\n");
		rc = f_write(&fileObj, header, strlen(header), &bw);
		break;
	default:
		break;
	}
	// Update file position.
	LOG_POSITIONS[log_type][index] += bw;

	if(rc != 0) DEBUGOUT("ERROR: %s f_write rc=%d\n", LOG_TYPE_STRINGS[log_type], rc);
	f_close_();
}

FRESULT f_open_log (LOG_TYPE log_type, int index, BYTE mode)
{
	if(!SDCARD_ACTIVE) return FR_OK;
	FRESULT rc;
	char filepath[32] = "";
	get_filepath(filepath, g_log_directory, log_type, index, TYPE_CSV);
	rc = f_open(&fileObj, filepath, mode);
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_open_log rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

FRESULT f_close_()
{
	if(!SDCARD_ACTIVE) return FR_OK;
	return f_close(&fileObj);
}

FRESULT f_lseek_(DWORD ofs)
{
	if(!SDCARD_ACTIVE) return FR_OK;
	return f_lseek(&fileObj, ofs);
}

FRESULT f_write_log(LOG_TYPE log_type, int index, char* data)
{
	if(!SDCARD_ACTIVE) return FR_OK;
	FRESULT rc;
	UINT bw;
	char data_[17];
	snprintf(data_, 17, "%s,", data);
	rc = f_write(&fileObj, data_, strlen(data_), &bw);
	LOG_POSITIONS[log_type][index] += bw;
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_write_log rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

FRESULT f_write_newline(LOG_TYPE log_type, int index)
{
	if(!SDCARD_ACTIVE) return FR_OK;
	FRESULT rc;
	UINT bw;
	rc = f_write(&fileObj, "\r\n", 2, &bw);
	LOG_POSITIONS[log_type][index] += bw;
	if(rc != 0) DEBUGOUT("ERROR: %s_%d f_write_newline rc=%d\n", LOG_TYPE_STRINGS[log_type], index, rc);
	return rc;
}

void get_filepath(char* filepath, char* dir, LOG_TYPE log_type, int index, char* filetype) {
	if(!SDCARD_ACTIVE) return;
	// Concat the dir and name variables and save to the filepath variable.
	if(strcmp("", dir) == 0) {
		sprintf(filepath, "%s_%d.%s", LOG_TYPE_STRINGS[log_type], index, filetype);
	}
	else {
		sprintf(filepath, "%s/%s_%d.%s", dir, LOG_TYPE_STRINGS[log_type], index, filetype);
	}
}
