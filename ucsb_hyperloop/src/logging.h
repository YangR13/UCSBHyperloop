#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>

#include "ethernet.h"
#include "subsystems.h"

typedef enum {
	LOG_LOGGING,
	LOG_POSITIONING_NAVIGATION,
	LOG_POWER_DISTRIBUTION,
	LOG_MAGLEV,
	LOG_BRAKING,
	LOG_SERV_PROP_PAYLOAD,
	NUM_LOGS,
} LOG_TYPE;

static const char LOG_TYPE_STRINGS[NUM_LOGS][8] = {
	"log"
	"pos",
	"pwr_d",
	"maglev",
	"brake"
	"sv_pyd"
};

typedef enum {
	MAX_EVT
} Event;

typedef enum {
	MAX_ERR
} Error;

// Logging.
#define LOG			"LOG"			// Log Collection
#define ERR			"ERR"			// Error Message

// Positioning & Navigation.
#define	POS			"POS"			// <GRAPH> Position X, Position Y, Position Z
#define	VEL			"VEL"			// <GRAPH> Velocity X, Velocity Y, Velocity Z
#define	ACL			"ACL"			// <GRAPH> Acceleration X, Acceleration Y, Acceleration Z
#define	RPY			"RPY"			// <GRAPH> Roll, Pitch, Yaw

// Power Distribution.
#define PD_BMS_VHL	"PD_BMS_VHL"	// Power Distribution BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low).
#define PD_BMS_CHG	"PD_BMS_CHG"	// <GRAPH-Percent> Power Distribution BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent).
#define PD_BMS_TH	"PD_BMS_TH"		// Power Distribution BMS Battery0 Temperature (High), Battery1 Temperature (High).

// Maglev.
#define	M_RPM 		"M_RPM"			// <GRAPH> Maglev0 RPM, Maglev1 RPM, Maglev2 RPM, Maglev3 RPM
#define	M_CUR		"M_CUR"			// Maglev0 Current, Maglev1 Current, Maglev2 Current, Maglev3 Current
#define	M_T			"M_T"			// Maglev0 Temperature (0, 1, 2, 3), Maglev1 Temperature (0, 1, 2, 3), Maglev2 Temperature (0, 1, 2, 3), Maglev3 Temperature (0, 1, 2, 3)
#define	M_BMS_F_VHL	"M_BMS_F_VHL"	// Maglev BMS Front Battery0 Voltage (High, Low), Battery1 Voltage (High, Low), Battery2 Voltage (High, Low)
#define	M_BMS_B_VHL	"M_BMS_B_VHL"	// Maglev BMS Back Battery0 Voltage (High, Low), Battery1 Voltage (High, Low), Battery2 Voltage (High, Low)
#define	M_BMS_F_CHG	"M_BMS_F_CHG"	// <GRAPH-Percent> Maglev BMS Front Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent), Battery2 Charge (Coulomb, Percent)
#define	M_BMS_B_CHG	"M_BMS_B_CHG"	// <GRAPH-Percent> Maglev BMS Back Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent), Battery2 Charge (Coulomb, Percent)
#define	M_BMS_F_TH	"M_BMS_F_TH"	// Maglev BMS Front Battery0 Temperature (High), Battery1 Temperature (High), Battery2 Temperature (High)
#define	M_BMS_B_TH	"M_BMS_B_TH"	// Maglev BMS Back Battery0 Temperature (High), Battery1 Temperature (High), Battery2 Temperature (High)

// Braking.
#define	B_F_POS		"B_F_POS"		// <SPECIAL> Braking Front Actuator0 Position, Actuator1 Position
#define	B_B_POS		"B_B_POS"		// <SPECIAL> Braking Back Actuator0 Position, Actuator1 Position
#define	B_F_CAL		"B_F_CAL"		// <SPECIAL> Braking Front Actuator0 Calibration (Disengaged, Ready, Engaged), Actuator1 Calibration (Disengaged, Ready, Engaged)
#define	B_B_CAL		"B_B_CAL"		// <SPECIAL> Braking Back Actuator0 Calibration (Disengaged, Ready, Engaged), Actuator1 Calibration (Disengaged, Ready, Engaged)
#define	B_F_T		"B_F_T"			// Braking Front Actuator0 Temperature (0, 1), Actuator1 Temperature (0, 1)
#define	B_B_T		"B_B_T"			// Braking Back Actuator0 Temperature (0, 1), Actuator1 Temperature (0, 1)
#define	B_BMS_VHL	"B_BMS_VHL"		// Braking BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low)
#define	B_BMS_CHG	"B_BMS_CHG"		// <GRAPH-Percent> Braking BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent)
#define	B_BMS_TH	"B_BMS_TH"		// Braking BMS Battery0 Temperature (High), Battery1 Temperature (High)

// Serv-Prop / Payload.
#define	SP_S		"SP_S"			// <SPECIAL>	Serv-Prop Actuator State (ENUM: 0-UNKNOWN, 1-LOWERED, 2-LOWERING, 3-RAISING, 4-RAISED), Payload Actuator State (ENUM: 0-UNKNOWN, 1-LOWERED, 2-LOWERING, 3-RAISING, 4-RAISED), Payload Motor State (ENUM: 0-UNKNOWN, 1-STATIONARY, 2-BACKWARDS, 3-FORWARDS)
#define	SP_T		"SP_T"			// Serv-Prop Actuator0 Temperature (0, 1), Serv-Prop Actuator1 Temperature (0, 1), Payload Actuator Temperature (0, 1), Payload Motor Temperature (0, 1)
#define	SP_BMS_VHL	"SP_BMS_VHL"	// Serv-Prop / Payload BMS Battery0 Voltage (High, Low), Battery1 Voltage (High, Low)
#define	SP_BMS_CHG	"SP_BMS_CHG"	// <GRAPH-Percent> Serv-Prop / Payload BMS Battery0 Charge (Coulomb, Percent), Battery1 Charge (Coulomb, Percent)
#define	SP_BMS_TH	"SP_BMS_TH"		// Serv-Prop / Payload BMS Battery0 Temperature (High), Battery1 Temperature (High)

void logAllData();
void logData(LOG_TYPE log_type);

void initEventLogFile();
void initErrorLogFile();
void logEventString(char* desc);
void logErrorString(char* desc);
void logStateMachineEvent(int sig);
void logEvent(Event evt);
void logError(Error err);

#endif // LOGGING_H_
