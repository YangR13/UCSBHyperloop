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
  {1, 0},   // {Hub, Port}; // TODO: Check me!
};
// TODO: Calibrated conversions reference voltage table for 18V5 BMS?

const uint8_t I2C_ADC_18V5_subBMS_Addresses[4] = {0x19, 0x0B, 0x18, 0xFF}; // TODO: What is address for 4th ADC?!?

// (Electronics) Power Distribution Board BMS Data:
const uint8_t PWR_DST_BMS_HUB_PORT[1][2] = {   // Only 1
  {0, 0},   // {Hub, Port}; // TODO: FIX ME
};
// TODO: Calibrated conversions reference voltage table for Power Distribution Board BMS?

const uint8_t I2C_ADC_PWR_DST_subBMS_Addresses[4] = {0x19, 0x0B}; // TODO: FIX ME

BMS_18V5* initialize_BMS_18V5(uint8_t identity) {
  BMS_18V5* bms = malloc(sizeof(BMS_18V5));
  bms->identity = identity;
  bms->bus = BMS_18V5_HUB_PORT[bms->identity][0];


  GPIO_Setup(HUB_AUX_GPIO_REGISTER[BMS_18V5_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[BMS_18V5_HUB_PORT[bms->identity][0]][BMS_18V5_HUB_PORT[bms->identity][1]], OUT);
  GPIO_Write(HUB_AUX_GPIO_REGISTER[BMS_18V5_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[BMS_18V5_HUB_PORT[bms->identity][0]][BMS_18V5_HUB_PORT[bms->identity][1]], 1);
  bms->relay_active_low = 1;

  int batt;
  for (batt = 0; batt < 4; batt++) {
    // Initialize battery voltages to a default value
    bms->battery_voltage[batt] = 9.25; // TODO: Is this a good starting value? 18.5V/2?

    // Initialize cell voltages to a default value
    int i = 0;
    for (i = 0; i < 5; i++) {
      bms->cell_voltages[batt][i] = 4.625; // TODO: Is this a good starting value? 9.25V / 5 cells?
    }

    // Initialize thermistor moving averages (with first read)
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      bms->temperatures[batt][temp_counter] = calculate_temperature(ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], temp_counter + 6));
    }
  }

  bms->amps[0] = 0;
  bms->amps[1] = 0;
  bms->timestamp = 0;
  bms->alarm = 0;
  return bms;
}

uint8_t update_BMS_18V5(BMS_18V5* bms) {
  int batt, i;
  float prev_voltage;

  //0x19 FLOAT LOW
  //0x0B FLOAT HIGH
  //0x18 FLOAT FLOAT
  for (batt = 0; batt < 4; batt++) {
    prev_voltage = 0;
    for (i = 0; i < 5; i++) {
      float voltage = ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], i) / MAX12BITVAL * 5.0 * MAGLEV_BMS_CAL_CONVERSIONS[bms->identity][batt][i];
      bms->cell_voltages[batt][i] = voltage - prev_voltage;
      prev_voltage = voltage;
    }
    bms->battery_voltage[batt] = prev_voltage;

    //Record Temperatures
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      int new_temperature = calculate_temperature(ADC_read(bms->bus, I2C_ADC_18V5_subBMS_Addresses[batt], temp_counter + 6));
      new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * bms->temperatures[batt][temp_counter]);
      bms->temperatures[batt][temp_counter] = new_temperature;
      if (new_temperature > BATT_MAX_TEMP || new_temperature < BATT_MIN_TEMP) {
        bms->alarm |= 0b00000001;
      }
    }

    // Get current readings
    // TODO:
    // READ FROM ADC CHANNEL FOR CURRENT SENSOR
    // CONVERT TO AMPS, SAVE TO BMS->amps[batt];
  }

  GPIO_Write(HUB_AUX_GPIO_REGISTER[MAGLEV_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[MAGLEV_BMS_HUB_PORT[bms->identity][0]][MAGLEV_BMS_HUB_PORT[bms->identity][1]], bms->relay_active_low);

  return bms->alarm;
}

// (Electronics) Power Distribution Board BMS Data:

PWR_DST_BMS* initialize_PWR_DST_BMS(uint8_t identity) {
    PWR_DST_BMS* bms = malloc(sizeof(PWR_DST_BMS));
  bms->identity = identity;
  bms->bus = PWR_DST_BMS_HUB_PORT[bms->identity][0];


  GPIO_Setup(HUB_AUX_GPIO_REGISTER[PWR_DST_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[PWR_DST_BMS_HUB_PORT[bms->identity][0]][PWR_DST_BMS_HUB_PORT[bms->identity][1]], OUT);
  GPIO_Write(HUB_AUX_GPIO_REGISTER[PWR_DST_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[PWR_DST_BMS_HUB_PORT[bms->identity][0]][PWR_DST_BMS_HUB_PORT[bms->identity][1]], 1);
  bms->relay_active_low = 1;

  int batt;
  for (batt = 0; batt < 4; batt++) {
    // Initialize battery voltages to a default value
    bms->battery_voltage[batt] = 9.25; // TODO: Is this a good starting value? 18.5V/2?

    // Initialize cell voltages to a default value
    int i = 0;
    for (i = 0; i < 5; i++) {
      bms->cell_voltages[batt][i] = 4.625; // TODO: Is this a good starting value? 9.25V / 5 cells?
    }

    // Initialize thermistor moving averages (with first read)
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      bms->temperatures[batt][temp_counter] = calculate_temperature(ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], temp_counter + 6));
    }
  }

  bms->amps[0] = 0;
  bms->amps[1] = 0;
  bms->timestamp = 0;
  bms->alarm = 0;
  return bms;
}

uint8_t update_PWR_DST_BMS_ACTIVE(PWR_DST_BMS* bms) {
  int batt, i;
  float prev_voltage;

  //0x19 FLOAT LOW
  //0x0B FLOAT HIGH
  //0x18 FLOAT FLOAT
  for (batt = 0; batt < 4; batt++) {
    prev_voltage = 0;
    for (i = 0; i < 5; i++) {
      float voltage = ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], i) / MAX12BITVAL * 5.0 * MAGLEV_BMS_CAL_CONVERSIONS[bms->identity][batt][i];
      bms->cell_voltages[batt][i] = voltage - prev_voltage;
      prev_voltage = voltage;
    }
    bms->battery_voltage[batt] = prev_voltage;

    //Record Temperatures
    int temp_counter = 0;
    for (temp_counter = 0; temp_counter < 2; temp_counter++) {
      int new_temperature = calculate_temperature(ADC_read(bms->bus, I2C_ADC_PWR_DST_subBMS_Addresses[batt], temp_counter + 6));
      new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * bms->temperatures[batt][temp_counter]);
      bms->temperatures[batt][temp_counter] = new_temperature;
      if (new_temperature > BATT_MAX_TEMP || new_temperature < BATT_MIN_TEMP) {
        bms->alarm |= 0b00000001;
      }
    }
  }

  GPIO_Write(HUB_AUX_GPIO_REGISTER[PWR_DST_BMS_HUB_PORT[bms->identity][0]], HUB_AUX_PINS[PWR_DST_BMS_HUB_PORT[bms->identity][0]][PWR_DST_BMS_HUB_PORT[bms->identity][1]], bms->relay_active_low);

  return bms->alarm;
}
