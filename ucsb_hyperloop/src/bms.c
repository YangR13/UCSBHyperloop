// BMS.c - Battery Management Systems
// 22V for Maglev
// 18.5V for Braking, Service Propulsion, and Payload Actuators
// 9V for Eletronics Power Distribution board

#include "bms.h"
#include "stdint.h"
#include "I2CPERIPHS.h"

// Maglev_BMS Data:
const uint8_t MAGLEV_BMS_HUB_PORT[NUM_MAGLEV_BMS][2] = {   //Max 1 per I2C bus or Hub
  {1, 0},   //{Hub, Port};
  {2, 0}
};
const float MAGLEV_BMS_CAL_CONVERSIONS[NUM_MAGLEV_BMS][3][6] = {
  { //BMS0
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99},   //SubBMS0
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99},   //SubBMS1
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99}    //SubBMS2
  },
  { //BMS1
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99},   //SubBMS0
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99},   //SubBMS1
    {2.0, 2.0, 3.0, 4.0, 5.02, 5.99}    //SubBMS2
  }
};
const uint8_t I2C_ADC_Maglev_subBMS_Addresses[3] = {0x19, 0x0B, 0x18};

// BMS_18V5 Data:
const uint8_t BMS_18V5_HUB_PORT[1][2] = {   // Only 1
  {0, 0},   // {Hub, Port}; // TODO: Check me! (Hub may be right but port is not)
};
// TODO: Calibrated conversions reference voltage table for 18V5 BMS?
const float BMS_18V5_CAL_5V0REF = 5.08;
const float BMS_18V5_CAL_3V3REF = 3.30;

// Addressing pins for ADCs: LOW-HIGH / FLOAT-HIGH / FLOAT-LOW / HIGH-LOW
const uint8_t I2C_ADC_18V5_subBMS_Addresses[4] = {0x0A, 0x0B, 0x19, 0x1A};

// (Electronics) Power Distribution Board BMS Data:
const uint8_t PWR_DST_BMS_HUB_PORT[1][2] = {   // Only 1
  {0, 3},   // {Hub, Port};
};
// TODO: Calibrated conversions reference voltage table for Power Distribution Board BMS?

// Addressing pins for ADCs: LOW-FLOAT / HIGH-FLOAT
const uint8_t I2C_ADC_PWR_DST_subBMS_Addresses[4] = {0x09, 0x1B};

// MAGLEV BMS

Maglev_BMS* initialize_Maglev_BMS(uint8_t identity) {
  Maglev_BMS* bms = malloc(sizeof(Maglev_BMS));
  bms->identity = identity;
  bms->bus = MAGLEV_BMS_HUB_PORT[bms->identity][0];


  GPIO_Setup(HUB_AUX_GPIO_REGISTER[MAGLEV_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[MAGLEV_BMS_HUB_PORT[bms->identity][0]][MAGLEV_BMS_HUB_PORT[bms->identity][1]], OUT);
  GPIO_Write(HUB_AUX_GPIO_REGISTER[MAGLEV_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[MAGLEV_BMS_HUB_PORT[bms->identity][0]][MAGLEV_BMS_HUB_PORT[bms->identity][1]], 1);
  bms->relay_active_low = 1;

  int batt;
  for (batt = 0; batt < 3; batt++) {
    // Initialize battery voltages to a default value
    bms->battery_voltage[batt] = 23.0; // TODO: Is this a good starting value? 23V?

    // Initialize cell voltages to a default value
    int i = 0;
    for (i = 0; i < 6; i++) {
      bms->cell_voltages[batt][i] = 3.833; // TODO: Is this a good starting value? 23V / 6 cells?
    }

    // Initialize thermistor moving averages (with first read)
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      bms->temperatures[batt][temp_counter] = calculate_temperature(ADC_read(bms->bus, I2C_ADC_Maglev_subBMS_Addresses[batt], temp_counter + 6));
    }
  }

  bms->amps = 0;
  bms->timestamp = 0;
  bms->alarm = 0;
  return bms;
}

uint8_t update_Maglev_BMS(Maglev_BMS* bms) {
  int batt, i;
  float prev_voltage;
  int new_alarms = 0b00;

  //0x19 FLOAT LOW
  //0x0B FLOAT HIGH
  //0x18 FLOAT FLOAT
  for (batt = 0; batt < 3; batt++) {
    prev_voltage = 0;
    for (i = 0; i < 6; i++) {
      float voltage = ADC_read(bms->bus, I2C_ADC_Maglev_subBMS_Addresses[batt], i) / MAX12BITVAL * 5.0 * MAGLEV_BMS_CAL_CONVERSIONS[bms->identity][batt][i];
      bms->cell_voltages[batt][i] = voltage - prev_voltage;
      prev_voltage = voltage;
    }
    bms->battery_voltage[batt] = prev_voltage;

    // Check cell voltages for a substantial imbalance, set an alarm if present
    float min = 9.9;
    float max = 0.0;
    for (i = 0; i < 5; i++){
        if (bms->cell_voltages[batt][i] < min){
            min = bms->cell_voltages[batt][i];
        }
        if (bms->cell_voltages[batt][i] > max){
            max = bms->cell_voltages[batt][i];
        }
    }
    if ((max - min) > MAX_MAGLEV_BATT_CELL_DELTA){
        new_alarms |= 0b10;
    }

    //Record Temperatures
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      int new_temperature = calculate_temperature(ADC_read(bms->bus, I2C_ADC_Maglev_subBMS_Addresses[batt], temp_counter + 6));
      new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * bms->temperatures[batt][temp_counter]);
      bms->temperatures[batt][temp_counter] = new_temperature;
      if (new_temperature > BATT_MAX_TEMP || new_temperature < BATT_MIN_TEMP) {
        new_alarms |= 0b01;
      }
    }
  }

  // Update alarm flags in the Maglev BMS struct
  // Unrecoverable faults are persistent, recoverable alarms are cleared
  // New alarms detected in this update cycle are always added
  if ((bms->alarm & 0b10) == 0b00){
      // No unrecoverable alarms previously existed, just save current alarms (clearing recoverable from previous cycles)
      bms->alarm = new_alarms;
  }
  else{
      // An unrecoverable fault previously existed, only update recoverable alarm bit of flag (bit 0)
      bms->alarm |= new_alarms;
  }

  GPIO_Write(HUB_AUX_GPIO_REGISTER[MAGLEV_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[MAGLEV_BMS_HUB_PORT[bms->identity][0]][MAGLEV_BMS_HUB_PORT[bms->identity][1]], bms->relay_active_low);

  return bms->alarm;
}

// 18.5V BMS

BMS_18V5* initialize_BMS_18V5(uint8_t identity) {
  BMS_18V5* bms = malloc(sizeof(BMS_18V5));
  bms->identity = identity;
  bms->bus = BMS_18V5_HUB_PORT[bms->identity][0];

  int batt;
  for (batt = 0; batt < 4; batt++) {
    // Initialize battery voltages to a default value
    bms->battery_voltage[batt] = 18.5; // TODO: Is this a good starting value? 18.5V?

    // Initialize cell voltages to a default value
    int i = 0;
    for (i = 0; i < 5; i++) {
      bms->cell_voltages[batt][i] = 3.7; // TODO: Is this a good starting value? 18.5V / 5 cells?
    }

    // Initialize thermistor moving averages (with first read)
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      bms->temperatures[batt][temp_counter] = calculate_temperature(ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], temp_counter + 5));
    }
  }

  bms->amps[0] = 0;
  bms->amps[1] = 0;
  bms->amps[2] = 0;
  bms->amps[3] = 0;
  bms->timestamp = 0;
  bms->alarm[0] = 0;
  bms->alarm[1] = 0;
  bms->alarm[2] = 0;
  return bms;
}

void update_BMS_18V5(BMS_18V5* bms) {
  int batt, i;
  float prev_voltage;
  // New alarms set this update cycle: 0 and 1 are braking pairs, 2 is service propulsion + payload actuators

  int new_alarms[3] = {0,0,0};

  for (batt = 0; batt < 4; batt++) {
    prev_voltage = 0;
    for (i = 0; i < 5; i++) {
      float voltage = ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], i) / MAX12BITVAL * 5.0 * MAGLEV_BMS_CAL_CONVERSIONS[bms->identity][batt][i];
      bms->cell_voltages[batt][i] = voltage - prev_voltage;
      prev_voltage = voltage;
    }
    bms->battery_voltage[batt] = prev_voltage;

    // Check cell voltages for a substantial imbalance, set an alarm if present
    float min = 9.9;
    float max = 0.0;
    int i;
    for (i = 0; i < 5; i++){
        if (bms->cell_voltages[batt][i] < min){
            min = bms->cell_voltages[batt][i];
        }
        if (bms->cell_voltages[batt][i] > max){
            max = bms->cell_voltages[batt][i];
        }
    }
    if ((max - min) > MAX_18V5_BATT_CELL_DELTA){
        if (batt < 2){
            new_alarms[0] |= 0b00000010; // Braking pair 1 alarm
            new_alarms[2] |= 0b00000010; // Service propulsion + payload actuators alarm
        }
        else{
            new_alarms[1] |= 0b00000010; // Braking pair 2 alarm
        }
    }

    //Record Temperatures
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      int new_temperature = calculate_temperature(ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], temp_counter + 5));
      new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * bms->temperatures[batt][temp_counter]);
      bms->temperatures[batt][temp_counter] = new_temperature;
      if (new_temperature > BATT_MAX_TEMP || new_temperature < BATT_MIN_TEMP) {
          if (batt < 2){
              new_alarms[0] |= 0b01; // Braking pair 1 alarm
              new_alarms[1] |= 0b01; // Service propulsion + payload actuators alarm

          }
          else{
              new_alarms[1] |= 0b01; // Braking pair 2 alarm
          }
       }
    }

    // Get current readings
    for (i = 0; i < 4; i++){
        // Read from ADC to get voltage
        uint16_t ammeter_ratio = ADC_read(bms->bus, I2C_ADC_Maglev_subBMS_Addresses[i], 7);
        uint8_t new_amps = 0.0;
        switch(i){
            case 0:
            case 3:{
                // ACS770 - Braking actuator pairs
                new_amps = abs(1000 * ammeter_ratio * BMS_18V5_CAL_5V0REF / MAX12BITVAL - 1000 * BMS_18V5_CAL_5V0REF / 2) / AMMETER_150A_SENSITIVITY; // Done in mV
                if (new_amps >= MAX_BRAKING_CURRENT){
                    new_alarms[i % 2] |= 0b01; // Save to indexes 0 or 1 (braking pairs)
                }
                break;
            }
            case 1:
            case 2:{
                // ACS759 - Service propulsion / Payload actuators
                new_amps = abs(1000 * ammeter_ratio * BMS_18V5_CAL_5V0REF / MAX12BITVAL - 1000 * BMS_18V5_CAL_3V3REF / 2) / AMMETER_50A_SENSITIVITY; // Done in mV
                // TODO: Check that the alarm doesn't get set by the 4th port not having any sensor plugged into it!!!
                if (new_amps >= MAX_SERV_PAYLOAD_CURRENT){
                    new_alarms[2] |= 0b01;
                }
                break;
            }
        }
        bms->amps[i] = new_amps;
     }
  }

  // Update alarm flags in the 18V5 BMS struct
  // Unrecoverable faults are persistent, recoverable alarms are cleared
  // New alarms detected in this update cycle are always added
  for (i = 0; i < 3; i++){
      if ((bms->alarm[i] & 0b10) == 0b00){
          // No unrecoverable alarms previously existed, just save current alarms (clearing recoverable from previous cycles)
          bms->alarm[i] = new_alarms[i];
      }
      else{
          // An unrecoverable fault previously existed, only update recoverable alarm bit of flag (bit 0)
          bms->alarm[i] |= new_alarms[i];
      }
  }

}

// POWER DISTRIBUTION BOARD BMS

PWR_DST_BMS* initialize_PWR_DST_BMS(uint8_t identity) {
  PWR_DST_BMS* bms = malloc(sizeof(PWR_DST_BMS));
  bms->identity = identity;
  bms->bus = PWR_DST_BMS_HUB_PORT[bms->identity][0];

  int batt = 0;
  for (batt = 0; batt < 2; batt++) {
    // Initialize battery voltages to a default value
    bms->battery_voltage[batt] = 18.5; // TODO: Is this a good starting value? 18.5V?

    // Initialize cell voltages to a default value
    int i = 0;
    for (i = 0; i < 5; i++) {
      bms->cell_voltages[batt][i] = 3.7; // TODO: Is this a good starting value? 18.5V / 5 cells?
    }

    // Initialize thermistor moving averages (with first read)
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      bms->temperatures[batt][temp_counter] = calculate_temperature(ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], temp_counter + 5));
    }
  }

  bms->timestamp = 0;
  bms->alarm = 0;
  return bms;
}

uint8_t update_PWR_DST_BMS_ACTIVE(PWR_DST_BMS* bms) {
  int batt, i;
  float prev_voltage;
  int new_alarms = 0b00;

  for (batt = 0; batt < 2; batt++) {
    prev_voltage = 0;
    for (i = 0; i < 5; i++) {
      float voltage = ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], i) / MAX12BITVAL * 5.0 * MAGLEV_BMS_CAL_CONVERSIONS[bms->identity][batt][i];
      bms->cell_voltages[batt][i] = voltage - prev_voltage;
      prev_voltage = voltage;
    }
    bms->battery_voltage[batt] = prev_voltage;

    // Check cell voltages for a substantial imbalance, set an alarm if present
    float min = 9.9;
    float max = 0.0;
    for (i = 0; i < 5; i++){
        if (bms->cell_voltages[batt][i] < min){
            min = bms->cell_voltages[batt][i];
        }
        if (bms->cell_voltages[batt][i] > max){
            max = bms->cell_voltages[batt][i];
        }
    }
    if ((max - min) > MAX_PWR_DST_BATT_CELL_DELTA){
        new_alarms |= 0b10;
    }

    //Record Temperatures
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      int new_temperature = calculate_temperature(ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], temp_counter + 5));
      new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * bms->temperatures[batt][temp_counter]);
      bms->temperatures[batt][temp_counter] = new_temperature;
      if (new_temperature > BATT_MAX_TEMP || new_temperature < BATT_MIN_TEMP) {
        new_alarms |= 0b01;
      }
    }
  }

  // Update alarm flags in the power distribution BMS struct
  // Unrecoverable faults are persistent, recoverable alarms are cleared
  // New alarms detected in this update cycle are always added
  if ((bms->alarm & 0b10) == 0b00){
      // No unrecoverable alarms previously existed, just save current alarms (clearing recoverable from previous cycles)
      bms->alarm = new_alarms;
  }
  else{
      // An unrecoverable fault previously existed, only update recoverable alarm bit of flag (bit 0)
      bms->alarm |= new_alarms;
  }

  return bms->alarm;
}
