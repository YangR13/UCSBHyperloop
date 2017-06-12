#include "actuators.h"
#include "pwm.h"
#include "I2CPERIPHS.h"

const uint8_t BOARD_I2C_BUS[NUM_ACTUATOR_BOARDS] = {2, 0, 0, 0};        // TODO: Set me
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
const uint8_t BOARD_PINS[16] =      {12, 14, 15, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// PWM pins on the PCB (data values) correspond to, in order:
//  Board 1 - PWM1, PWM2 | Board 2 (same), etc.
const LPC_PWM_T * BOARD_PWM_PORTS[8] = {LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1};
const uint8_t BOARD_PWM_CHANNELS[8] = {6, 5, 4, 3, 2, 1, 1, 1};
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
  board->position[0] = ADC_read(board->bus, board->ADC_device_address, 5);
  board->position[1] = ADC_read(board->bus, board->ADC_device_address, 1);

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
  board->target_pos[0] = 0;
  board->target_pos[1] = 0;

  // Write default values to GPIO/PWM pins
  int output_counter = 0;
  for (output_counter = 0; output_counter < 2; output_counter++){
      // Direction signal
      GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + output_counter], BOARD_PINS[(board->identity * 4) + output_counter], board->direction[output_counter]);
      // PMW enable/speed signal
      PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], board->enable[output_counter]);
  }

  // TODO: Remove hack
  current_reading = 0.0;

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
      board->bridge_fault[0] = !GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 2], BOARD_PINS[(board->identity * 4) + 2]);
      board->bridge_fault[1] = !GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 3], BOARD_PINS[(board->identity * 4) + 3]);
  }

  //return board->alarm;
  return 0;
}

// TODO: Change the usable stroke length to a realistic value!
#define USABLE_STROKE_LEN 2000.0
#define MIN_DUTY_CYCLE 0.05

void move(ACTUATORS *board, int num, int destination){
    // Begin the routine for an actuator to move to a target position

    // Do an off-cycle update to make sure that the position feedback data is not stale
    update_actuator_control(board);

    // Set the target destination value
    board->target_pos[num] = destination;
    if (destination == -1){
        board->direction[num] = 1;
        board->enable[num] = 0.10;
    }
    else if (destination == -2){
        board->direction[num] = 0;
        board->enable[num] = 0.10;
    }

    // Calculate the target direction and duty cycle
    // Must be called here because otherwise ->enable is currently 0 and will be blocked by the guard in update_actuator_board
    calculate_actuator_control(board, num);

    // Then update to write the calculated control signals to the board
    update_actuator_board(board);
}

void calculate_actuator_control(ACTUATORS *board, int num){
    if (board->target_pos[num] < 0){
        return;
    }

    // Check to see if the destination has been reached (and movement should be stopped)
    if (abs(board->position[num] - board->target_pos[num]) <= 2){
        // Stop movement (and write to the PWM output immediately to prevent overshoot)
        board->enable[num] = 0.0;
        PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + num], BOARD_PWM_CHANNELS[(board->identity * 2) + num], board->enable[num]);
        DEBUGOUT("Target reached! Target: %d, Actual: %d\n", board->target_pos[num], board->position[num]);
    }
    else{
        // Determine direction: forwards = 1, backwards = 0
        board->direction[num] = board->position[num] > board->target_pos[num];

        // Otherwise determine the updated duty cycle to drive the actuator at
        // Percentage of usable stroke length to travel => percentage of duty cycle to use (with minimum at 5%)
        float pct_away = (float)abs(board->position[num] - board->target_pos[num]) / USABLE_STROKE_LEN;
        float cycle = 1;
        if (pct_away < 1.0){
            // 5th-power relation between distance to travel and output duty cycle
            cycle -= pow((1 - pct_away), 3);
        }

        if (cycle < MIN_DUTY_CYCLE){
            cycle = MIN_DUTY_CYCLE;
        }
        board->enable[num] = cycle;
        DEBUGOUT("Output duty cycle is %f \n", cycle);
    }
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
#define AVG_ALPHA 0.50

    // Read and process position feedback
    //  Calculate a moving average using only those values not considered erroneous
    int pos_counter = 0;
    for (pos_counter = 0; pos_counter < 2; pos_counter++){
        uint16_t pos = ADC_read(board->bus, board->ADC_device_address, 5 - (4 * pos_counter));
        // THIS IS A TEMP HACK
        if (pos_counter == 1){
            current_reading = (0.5 * (((float)pos - 1322.0) * (5000.0 / MAX12BITVAL) / 8.7)) + 0.5 * current_reading;
            DEBUGOUT("%f \n", current_reading);
        }
        else{
            board->position[pos_counter] = (AVG_ALPHA * pos) + ((1 - AVG_ALPHA) * board->position[pos_counter]);
        }

        // Service actuator movement routine if it is currently active
        if (board->enable[pos_counter] >= MIN_DUTY_CYCLE){
            calculate_actuator_control(board, pos_counter);
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
