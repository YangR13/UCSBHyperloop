#include "actuators.h"
#include "pwm.h"
#include "I2CPERIPHS.h"

const uint8_t BOARD_I2C_BUS[NUM_ACTUATOR_BOARDS] = {1, 0, 0, 0};        // TODO: Set me
const uint8_t BOARD_I2C_DIP[NUM_ACTUATOR_BOARDS] = {0,  255,  0,  255}; // TODO: Change to realistic values!
#ifdef ARDUINO
// Pin values to use for GPIO and PWM signals
const uint8_t BOARD_PINS[16] =      {52, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PWM_CHANNELS[8] = {2, 3, 0, 0, 0, 0, 0, 0};

// N/A for Arduino but provided for use of LPC-compatible PWM and GPIO functions.
typedef struct {
    int dummy_value.
} LPC_PWM_T;
const LPC_PWM_T * BOARD_PWM_PORTS[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PIN_PORTS[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#endif
#ifdef LPC
// TODO: Change all of these to realistic pin values!

// GPIO pins on the PCB (data values) correspond to, in order:
//  Board 1 - DIR1, DIR2, FAULT1, FAULT2 | Board 2 (same) | etc.
const uint8_t BOARD_PIN_PORTS[16] = {3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PINS[16] =      {12, 12, 13, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// PWM pins on the PCB (data values) correspond to, in order:
//  Board 1 - PWM1, PWM2 | Board 2 (same), etc.
const LPC_PWM_T * BOARD_PWM_PORTS[8] = {LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1};
const uint8_t BOARD_PWM_CHANNELS[8] = {5, 6, 4, 3, 2, 1, 1, 1};
#endif

// This can't be imported from I2CPERIPHS because it's in the .c and not the .h.
const uint8_t ADC_Address_Select_Actuators[4] = {0x8, 0xA, 0x1A, 0x28};

ACTUATORS* initialize_actuator_board(uint8_t identity) {
  ACTUATORS* board = malloc(sizeof(ACTUATORS));
  board->identity = identity;
  board->bus = BOARD_I2C_BUS[board->identity];

  // TODO: Check if this works right for boards #2 and #3, especially with #0 and #1 present on the same bus.
  board->ADC_device_address = ADC_Address_Select_Actuators[board->identity];

  // Initialize thermistor moving averages (with first read)
  int temp_counter;
  for (temp_counter = 0; temp_counter < 4; temp_counter++) {
    board->temperatures[temp_counter] = calculate_temperature(ADC_read(board->bus, board->ADC_device_address, 6 - (2 * temp_counter)));
  }

  // Initialize current readings
  int amps_counter;
  for (amps_counter = 0; amps_counter < 2; amps_counter++){
      board->amps[amps_counter] = 0;
  }

  // Initialize H-bridge fault indicators
  int fault_counter;
  for (fault_counter = 0; fault_counter < 2; fault_counter++){
      board->bridge_fault[fault_counter] = 0;
  }

  // Initialize position variables (with first read)
  board->position[0] = ADC_read(board->bus, board->ADC_device_address, 1);
  board->position[1] = ADC_read(board->bus, board->ADC_device_address, 5);

  // Initialize GPIO/PWM pins
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 0], BOARD_PINS[(board->identity * 4) + 0], OUT); // Direction 1
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 1], BOARD_PINS[(board->identity * 4) + 1], OUT); // Direction 2
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 2], BOARD_PINS[(board->identity * 4) + 2], IN); // H-bridge 1 fault
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 3], BOARD_PINS[(board->identity * 4) + 3], IN); // H-bridge 2 fault
  PWM_Setup(BOARD_PWM_PORTS[(board->identity * 2) + 0], BOARD_PWM_CHANNELS[(board->identity * 2) + 0]); // PWM output 1
  PWM_Setup(BOARD_PWM_PORTS[(board->identity * 2) + 1], BOARD_PWM_CHANNELS[(board->identity * 2) + 1]); // PWM output 2

  // Initialize control values
  board->direction[0] = 1;  // Forward
  board->direction[1] = 1;  // Forward
  board->enable[0] = 0;     // Stopped
  board->enable[1] = 0;     // Stopped

  // Write default values to GPIO/PWM pins
  int output_counter = 0;
  for (output_counter = 0; output_counter < 2; output_counter++){
      // Direction signal
      GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + output_counter], BOARD_PINS[(board->identity * 4) + output_counter], board->direction[output_counter]);
      // PMW enable/speed signal
      PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], board->enable[output_counter]);
  }

  return board;
}

uint8_t update_actuator_board(ACTUATORS* board) {
    // Gathers sensor data from a braking board
    // Temperature, current, and H-bridge fault signals.
    // Actuation signals and position feedback are updated in update_actuator_control()
    update_actuator_control(board);

    // Record Temperatures
    int temp_counter;
    for (temp_counter = 0; temp_counter < 4; temp_counter++) {
      board->temperatures[temp_counter] = calculate_temperature(ADC_read(board->bus, board->ADC_device_address, 6 - (2 * temp_counter)));
      /*
        if (new_temperature > HEMS_MAX_TEMP || new_temperature < HEMS_MIN_TEMP){
          board->alarm |= 0b00000001;
        }
      */
    }

  // Record Motor Controller Currents
  // H-bridge current sensors output: (50mV offset) + (20 mV / amp)
  int current_counter = 0;
  for (current_counter = 0; current_counter < 2; current_counter++){
      uint16_t raw_reading = ADC_read(board->bus, board->ADC_device_address, 7 - (4 * current_counter));
      uint16_t reading = ((((float)raw_reading * 5.0 / MAX12BITVAL) - 0.050) / 0.020); // Ratio -> volts -> amps
      board->amps[current_counter] = reading;
      /*
      if (new_amps > HEMS_MAX_CURRENT){
          board->alarm |= 0b00000010;
      }
      */
  }

  // Read fault signals from H-bridges (GPIO pins)
  int fault_counter = 0;
  for (fault_counter = 0; fault_counter < 2; fault_counter++){
      board->bridge_fault[0] = GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 2], BOARD_PINS[(board->identity * 4) + 2]);
      board->bridge_fault[1] = GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 3], BOARD_PINS[(board->identity * 4) + 3]);
  }

  //return board->alarm;
  return 0;
}

void update_actuator_control(ACTUATORS *board){
    // Update data relevant to braking actuator movement
    //  This function is separated to allow for fast sampling of position feedback
    // Read position feedback signal from linear potentiometer -> ADC
    // Write direction signal to GPIO pins, set output PWM frequency

// Position values that are greater than this percentage away from the previous moving average
//  are considered erroneous and are discarded
#define MAX_POSITION_DIFF_PCT 0.30
// Alpha value for moving average window
#define AVG_ALPHA 0.02

    // Read and process position feedback
    //  Calculate a moving average using only those values not considered erroneous
    int pos_counter = 0;
    for (pos_counter = 0; pos_counter < 2; pos_counter++){
        uint16_t pos = ADC_read(board->bus, board->ADC_device_address, (4 * pos_counter) + 1);
        if (abs(pos - board->position[pos_counter]) < (MAX_POSITION_DIFF_PCT * board->position[pos_counter])){
            board->position[pos_counter] = (AVG_ALPHA * pos) + ((1 - AVG_ALPHA) * board->position[pos_counter]);
        }
    }

    // Update direction and PWM
    int output_counter = 0;
    for (output_counter = 0; output_counter < 2; output_counter++){
        // Direction signal
        GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + output_counter], BOARD_PINS[(board->identity * 4) + output_counter], board->direction[output_counter]);
        // PMW enable/speed signal
        PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], board->enable[output_counter]);
    }
}

void PWM_Setup(const void * pwm, uint8_t pin){
#ifdef ARDUINO
    pinMode(pin, OUTPUT);
#endif
#ifdef LPC
    Init_Channel((LPC_PWM_T *) pwm, pin);
#endif
}

void PWM_Write(const void * pwm, uint8_t pin, float duty) {
#ifdef ARDUINO
    analogWrite(AUX_PIN, setting);
#endif // ARDUINO
#ifdef LPC
    Set_Channel_PWM((LPC_PWM_T *) pwm, pin, duty);
#endif // LPC
}
