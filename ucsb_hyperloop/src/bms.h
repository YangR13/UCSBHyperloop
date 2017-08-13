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

// MAGLEV BMS

typedef struct{   //Designed for 3x 6S batteries;
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                //Only one allowed per bus, since addresses are hard-wired.

  //Data Storage
  float battery_voltage[3];   //From left to right on the board.
  float battery_charge_coulomb[3];
  float battery_charge_percent[3];
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
  float battery_charge_coulomb[4];
  float battery_charge_percent[4];
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
  float battery_charge_coulomb[2];
  float battery_charge_percent[2];
  float cell_voltages[2][5];
  int temperatures[2][2];

  float timestamp;
  uint8_t alarm;
} PWR_DST_BMS;

PWR_DST_BMS* initialize_PWR_DST_BMS(uint8_t identity);
uint8_t update_PWR_DST_BMS(PWR_DST_BMS* bms);

#endif
