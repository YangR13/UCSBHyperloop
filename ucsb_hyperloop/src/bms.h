#ifndef BMS_H
#define BMS_H

#include "stdint.h"

// GENERAL INFO
#define NUM_MAGLEV_BMS 2
#define NUM_ELECTRONICS_BMS 1
#define NUM_18V5_BMS 2
#define BATT_MAX_TEMP 60      //Too hot
#define BATT_MIN_TEMP 5

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
  uint8_t amps[4];            // 4 current sensors, one for each battery

  //Controls
  uint8_t relay_active_low;       //Active Low

  float timestamp;
  uint8_t alarm;
} BMS_18V5;

BMS_18V5* initialize_BMS_18V5(uint8_t identity);
uint8_t update_BMS_18V5(BMS_18V5* bms);



#endif
