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

// Safety:
#define ACTUATOR_MAX_TEMP 60      // Too hot
#define ACTUATOR_MIN_TEMP 5       // More to detect disconnects than for "too cold"
#define ACTUATOR_MAX_CURRENT 50

#define MAX12BITVAL 4095.0

// Control
#define TIME_CONTROL_PWM_DUTY 0.50

float current_reading;

#define IN 0
#define OUT 1

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

  // Variables for control
  int16_t target_pos[2];
  uint32_t times[2][2]; // For each actuator, start time of movement and how long to move

} ACTUATORS;

ACTUATORS* initialize_actuator_board(uint8_t identity);
uint8_t update_actuator_board(ACTUATORS* board);
void update_actuator_control(ACTUATORS *board);
int calculate_temperature(uint16_t therm_adc_val);
void move_time(ACTUATORS *board, int num, int dir, int interval);
void move_to_pos(ACTUATORS * board, int num, int destination);
void calculate_actuator_control(ACTUATORS *board, int num);


// The PWM channels are defined in a const array so the parameters are const accordingly.
void PWM_Setup(const void * pwm, uint8_t pin);
void PWM_Write(const void * pwm, uint8_t pin, float duty);

// See I2CPERIPHS.h for details on I2C ADC addressing


#endif //ACTUATORS_H
