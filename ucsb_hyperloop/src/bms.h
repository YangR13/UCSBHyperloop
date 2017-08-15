#ifndef BMS_H
#define BMS_H

#include "stdint.h"

// GENERAL INFO
#define NUM_MAGLEV_BMS 2
#define NUM_ELECTRONICS_BMS 1
#define NUM_18V5_BMS 2
#define BATT_MAX_TEMP 60      //Too hot
#define BATT_MIN_TEMP 5
#define MIN_CELL_VOLTS 3.4 // TODO: Verify this
#define MAX_MAGLEV_BATT_CELL_DELTA 0.37    // TODO: Is 10% of normal cell voltage a valid threshold?!
#define MAX_18V5_BATT_CELL_DELTA 0.37    // TODO: Is 10% of normal cell voltage a valid threshold?!
#define MAX_PWR_DST_BATT_CELL_DELTA 0.37    // TODO: Is 10% of normal cell voltage a valid threshold?!
#define MAX_MAGLEV_CURRENT 50.0 // TODO: Set realistic max maglev current reading
#define MAX_BRAKING_CURRENT 50.0 // TODO: Set realistic max braking current reading
#define MAX_SERV_PAYLOAD_CURRENT 30.0 // Todo: Set realistic max service propulsion + payload actuator current reading

// INFO ON FAULTS
// Each BMS struct has an alarm variable for each chain of batteries wired in series which are connected to it
// Recoverable faults are saved to that variable as '1' - excessive temperature or current draw, etc.
// Unrecoverable faults are saved to that variable as '2' - excessive cell voltage imblanaces, etc.

//LUT for 18.5V batteries
//Battery voltage -> Battery Charge (16.3V to 19.8V, .1V increments)

static const float voltageToCharge18_5LUT[] =
{
	13791.3, 13822.7, 13859.65, 13905.25, 13943.2, 13984, 14034.65, 14087.75, 14146.05,
	14211.9, 14277.75, 14348.65, 14424.6, 14510.7, 14601.85, 14698.1, 14814.55, 14936.1,
	15072.85, 15295.65, 16146.45, 16987.15, 18000, 19266.05, 20430.8, 21610.75, 22699.55,
	23671.85, 24547.9, 25256.9, 25849.35, 26381.05, 26811.5, 27226.75, 27576.15, 27702.2
};

//LUT for 22V batteries
//Battery voltage -> Battery Charge (20.0V to 23.7V, .1V increments)

static const float voltageToCharge22LUT[] =
{
	28566.235, 28632.175, 28692.625, 28769.065, 28843.62, 28924.33, 29018.13, 29120.54,
	29220.36, 29335.37, 29458.64, 29584.5, 29726.04, 29881.44, 30047.27, 30215.62,
	30412.74, 30629.74, 30863.26, 31187.08, 31999.64, 34539.8, 36000, 37905.4, 40436.6,
	43561.61, 46687.46, 49542.935, 52385.575, 54462.195, 56028.445, 57444.055, 58527.445,
	59417.915, 60400.435, 61432.025, 62240.345, 62317.385
};

// MAGLEV BMS

typedef struct{   //Designed for 3x 6S batteries;
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                //Only one allowed per bus, since addresses are hard-wired.

  //Data Storage
  float battery_voltage[3];   //From left to right on the board.
  float cell_voltages[3][6];
  int temperatures[3][2];
  uint8_t amps;               //No onboard ammeter; relies on data from HEMS or other.

  //Controls
  uint8_t relay_active_low;       //Active Low

  float timestamp;
  uint8_t alarm;
} Maglev_BMS;

Maglev_BMS* initialize_Maglev_BMS(uint8_t identity);
uint8_t update_Maglev_BMS(Maglev_BMS* bms);

// 18V5 BMS

typedef struct{   // Designed for 4x 5S batteries;
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                // Only one per bus

  //Data Storage
  float battery_voltage[4];   // From left to right on the board.
  float cell_voltages[4][5];
  int temperatures[4][2];
  uint8_t amps[4];            // 4 current sensor ports, #1 and #2 currently unused.

  float timestamp;
  uint8_t alarm[3];
} BMS_18V5;

BMS_18V5* initialize_BMS_18V5(uint8_t identity);
void update_BMS_18V5(BMS_18V5* bms);

// (Electronics) Power Distribution Board BMS

typedef struct{   // Designed for 4x 5S batteries;
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                // Only one per bus

  //Data Storage
  float battery_voltage[2];   // From left to right on the board.
  float cell_voltages[2][5];
  int temperatures[2][2];

  float timestamp;
  uint8_t alarm;
} PWR_DST_BMS;

PWR_DST_BMS* initialize_PWR_DST_BMS(uint8_t identity);
uint8_t update_PWR_DST_BMS(PWR_DST_BMS* bms);

#endif
