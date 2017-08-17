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

enum ACTUATOR_BOARDS {
	ACTUATOR_BOARD_BRAKING_0,
	ACTUATOR_BOARD_BRAKING_1,
	ACTUATOR_BOARD_PAYLOAD,
	ACTUATOR_BOARD_SERVICE_PROPULSION,
};
#define ACTUATOR_BOARD_BRAKING_MIN ACTUATOR_BOARD_BRAKING_0
#define ACTUATOR_BOARD_BRAKING_MAX ACTUATOR_BOARD_BRAKING_1

// Actuator disengaged starting positions.
static const float BRAKING_DISENGAGED_POSITIONS[2][2] = {
	{ 3700, 3700 },	// Front.
	{ 3700, 3700 }	// Back.
};

#define READY_OFFET_FROM_ENGAGED	250

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
#define MIN_DUTY_CYCLE 0.13
#define MIN_PWM_MODE_DUTY_CYCLE (MIN_DUTY_CYCLE - 0.01)
#define MIN_BWD_DUTY_CYCLE 0.08
#define MAX_FWD_DUTY_CYCLE 0.4
#define MAX_BWD_DUTY_CYCLE 0.4

#define USABLE_STROKE_LEN 2000.0 // TODO: Change the usable stroke length to a realistic value!

// TODO: SET THESE TO REALISTIC VALUES
#define PAYLOAD_RAISE_TIME 1000 // ms
#define PAYLOAD_RAISE_PWM 0.25
#define PAYLOAD_LOWER_TIME 1000 // ms
#define PAYLOAD_LOWER_PWM 0.25
#define SERVICE_PROP_LOWER_TIME 20000 // ms
#define SERVICE_PROP_LOWER_PWM 0.25
#define SERVICE_PROP_RAISE_TIME 20000 // ms
#define SERVICE_PROP_RAISE_PWM 0.25
#define SERVICE_PROP_DRIVE_PWM 0.25

#define POS_MOV_AVG_ALPHA 0.50 // Alpha for position feedback moving average
#define POS_MAX_DELTA 300 // Readings greater than this delta away from the current moving average are considered erroneous.

#define STALL_CYCLES_STOP 100 // Number of update cycles where actuator feedback hasn't changed before stopping actuator.
#define STALL_CYCLES_PWM_INCREASE 10
#define STALL_CYCLES_LAST_PWM_MULTIPLIER 10
#define MOVE_CYCLE_PWM_DECREASE 0.01

#define IN 0
#define OUT 1

// Enum for when to stop an actuator
enum stop_modes{
    NO_STOP,
    STOP_FROM_TIME,
    STOP_FROM_POSITION,
    STOP_FROM_PWM_STALL
};

enum calibration_steps_enum {
	CALIBRATION_DONE,
	CALIBRATION_INIT,
	CALIBRATION_APPROACH,
	CALIBRATION_MOVE_TO_PWM,
	CALIBRATION_DEINIT,
};

enum engage_steps_enum {
	ENGAGE_DONE,
	ENGAGE_MOVE_TO_CHECKPOINT,
	ENGAGE_MOVE_TO_TARGET,
};

int calibration_step;

typedef struct {
  uint8_t identity;

  //I2C Parameters
  uint8_t bus;                  // Which I2C bus
  uint8_t ADC_device_address;   // ADC LTC2309 - Thermistors, Ammeter

  //Data Storage
  int temperatures[4];
  uint16_t amps[2];             // From H-bridges

  // Safety
  uint8_t bridge_fault[2];
  uint8_t alarm[2];
  uint8_t faulted;
  uint8_t pos_fault[2];
  uint8_t consecutive_pos_faults[2];
  uint8_t consecutive_identical_pos_faults[2];
  uint16_t previous_invalid_pos[2];

  // Output control signals
  uint8_t direction[2];         // 0 = backwards, 1 forwards
  uint8_t enable[2];            // Whether PWM output is enabled or disabled and set to 0
  float pwm[2];                 // PWM - 0 is none, 1.0 is full cycle

  // Applicable for all actuators - move by time only
  uint32_t times[2][2];         // For each actuator, start time of movement ([0]) and how long to move ([1])
  uint8_t has_feedback;          // 0 -> service propulsion and payload actuator boards. 1 -> braking boards.

  // BRAKING ACTUATORS ONLY
  uint16_t position[2];         // Positioning feedback data from linear potentiometers
  uint8_t stop_mode[2];         // Stop mode 0 -> none, stop mode 1 -> time, stop mode 2 -> position, stop mode 3 -> stall at PWM
  int16_t target_pos[2];        // Go to this point
  uint8_t pwm_algorithm[2];     // 0 -> pwm based on whether actuator is successfully moving, 1 -> exponential algorithm
  float target_pwm[2];          // In target PWM mode, go until position movement stalls at *this* target PWM
  uint16_t stalled_cycles[2];   // Number of cycles since position feedback has changed
  uint16_t prev_position[2];    // Previous position feedback value
  int16_t calibrated_engaged_pos[2];
  int engage_step;

} ACTUATORS;

ACTUATORS *braking_boards[2]; // 2 Braking boards.
ACTUATORS *service_prop;
ACTUATORS *payload;

ACTUATORS* initialize_actuator_board(uint8_t identity);
uint8_t update_actuator_board(ACTUATORS* board);
void update_actuator_control(ACTUATORS *board);
int calculate_temperature(uint16_t therm_adc_val);
void move_time(ACTUATORS *board, int num, int dir, int interval, float pwm);
void move_to_pos(ACTUATORS *board, int num, int destination);
void move_in_dir(ACTUATORS *board, int num, int dir, float pwm);
void move_to_disengaged_pos(ACTUATORS *board, int num);
void move_to_pwm(ACTUATORS *board, int num, int dir, float pwm);
void calculate_actuator_control(ACTUATORS *board, int num);

// Functions for all actuators.
void actuator_pair_stop(ACTUATORS *board);
void actuator_single_stop(ACTUATORS *board, int num);

// Braking actuator specific functions.
void start_actuator_calibration();
void update_actuator_calibration(ACTUATORS *board);
void start_actuator_engage(ACTUATORS *board);
void update_actuator_engage(ACTUATORS *board);
void start_actuator_ready(ACTUATORS *board);
void start_actuator_disengage(ACTUATORS *board);

// Service propulsion specific functions.
void servprop_raise(ACTUATORS *board);
void servprop_lower(ACTUATORS *board);
void servprop_drive_forwards(ACTUATORS *board);
void servprop_drive_backwards(ACTUATORS *board);

// Payload specific functions.
void payload_raise(ACTUATORS *board);
void payload_lower(ACTUATORS *board);


// The PWM channels are defined in a const array so the parameters are const accordingly.
void PWM_Setup(const void * pwm, uint8_t pin);
void PWM_Write(const void * pwm, uint8_t pin, float duty);

// See I2CPERIPHS.h for details on I2C ADC addressing

#endif //ACTUATORS_H
