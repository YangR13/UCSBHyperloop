// Actuators

#ifndef ACUTATORS_H_
#define ACTUATORS_H_

//#define ARDUINO
#define LPC

//Includes and Libraries
#include "math.h"

#ifdef ARDUINO //ARDUINO LIBRARIES BELOW
#include "Arduino.h"
#include "Wire.h"
#endif //ARDUINO

#ifdef LPC //LPC LIBRARIES BELOW
#include "stdlib.h"
#include "initialization.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#endif //LPC


// Number of Boards
#define NUM_ACTUATORS 2 // Per board
#define NUM_THERMISTORS 4 // Per board
#define NUM_ACTUATOR_BOARDS 4

// Thermistor Data
#define REFERENCE_RESISTANCE 5100 //[ohms]
#define THERMISTOR_BETA 3380
#define THERMISTOR_OFFSET -2.126

// Ammeter Data
#define AMMETER_10A_SENSITIVITY 264
#define AMMETER_50A_SENSITIVITY 40
#define AMMETER_150A_SENSITIVITY 8.8        //[mV/A] for the 150amp version of the sensor

//Safety:
#define ACTUATOR_MAX_TEMP 60      // Too hot
#define ACTUATOR_MIN_TEMP 5       // More to detect disconnects than for "too cold"
#define ACTUATOR_MAX_CURRENT 50

//Averaging:
#define TACHOMETER_AVG_WEIGHT 0.2 //Out of 1 (value = (old_value * AVG_WEIGHT + (1 - AVG_WEIGHT) * new_value) Set to 0 if you don't want exponential averaging.
#define THERMISTOR_AVG_WEIGHT 0.4 //Out of 1 (value = (old_value * AVG_WEIGHT + (1 - AVG_WEIGHT) * new_value)

#define MAX12BITVAL 4095.0

#define IN 0
#define OUT 1
void GPIO_Setup(uint8_t port, uint8_t pin, uint8_t dir);
void GPIO_Write(uint8_t port, uint8_t pin, uint8_t setting);
uint8_t GPIO_Read(uint8_t port, uint8_t pin);


// These are const to make the compiler happy.
void PWM_Setup(const void * pwm, uint8_t pin);
void PWM_Write(const void * pwm, uint8_t pin, float duty);

typedef struct {
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                     //Which I2C bus
  uint8_t ADC_device_address;   //ADC LTC2309 - Thermistors, Ammeter

  //Data Storage
  int temperatures[4];
  uint16_t amps[2];
  uint16_t position[2];

  //Safety
  uint8_t bridge_fault[2];

  // Output control signals
  uint8_t direction[2]; // 0 = backwards, 1 forwards
  float enable[2]; // PWM - 0 is none, 1.0 is full cycle

} ACTUATORS;

ACTUATORS* initialize_actuator_board(uint8_t identity);
uint8_t update_actuator_board(ACTUATORS* board);
void update_actuator_control(ACTUATORS *board);
int calculate_temperature(uint16_t therm_adc_val);

/*HEMS I2C Parameters
  Device Addressing (7-bit addressing):
  ADC LTC2309: 0 ...                                  -0??10??-   //Tri-state inputs A0 and A1, however we'll not use float (don't need that many addresses)
  I2C_DIP: 0b?????XXX   //X = don't cares; can be anything. They're not connected.
*/

/*ADC LTC2309
   Max I2C Clock Frequency: 400kHz
  I2C Protocol:
    1. Master Write: Device Address (with Write Bit)
    2. Master Write: DIN (Input Data Word)
    3. Master Write: Device Address (with Read Bit)
    4. Master Read: 2 Bytes
  Input Data Word (6-bit)
  S/D | O/S | S1 | S0 | UNI | SLP | X | X
  S/D = Single-Ended/_Differential (We'll always use Single-Ended here, so = 1)
  O/S = Odd/_Sign (Used to select which channel)
  S1 = Channel Select Bit 1 (Also used to select which channel)
  S2 = Channel Select Bit 0 (Also used to select which channel)
  UNI = Unipolar/_Bipolar (We'll always use Unipolar, so = 1)
  SLP = Sleep Mode (We won't put this into sleep mode, so = 0)
*/
#define ADC_CONFIG 0x88     //0b1???10XX

// Single-Ended Channel Configuration
#define LTC2309_CHN_0   0x80
#define LTC2309_CHN_1   0xC0
#define LTC2309_CHN_2   0x90
#define LTC2309_CHN_3   0xD0
#define LTC2309_CHN_4   0xA0
#define LTC2309_CHN_5   0xE0
#define LTC2309_CHN_6   0xB0
#define LTC2309_CHN_7   0xF0

//ADC Associated Functions:
//uint16_t ADC_read_actuators(uint8_t i2c_bus, uint8_t ADC_address, uint8_t ADC_channel);
uint16_t ADC_read(uint8_t i2c_bus, uint8_t ADC_address, uint8_t ADC_channel);


#endif //ACTUATORS_H
