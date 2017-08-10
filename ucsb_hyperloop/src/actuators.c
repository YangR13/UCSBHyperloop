#include "actuators.h"
#include "pwm.h"
#include "I2CPERIPHS.h"

const uint8_t BOARD_I2C_BUS[NUM_ACTUATOR_BOARDS] = {0, 0, 0, 0};        // TODO: Set me
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
const uint8_t BOARD_PIN_PORTS[16] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
const uint8_t BOARD_PINS[16] =      {7, 5, 6, 4, 3, 1, 2, 0, 15, 13, 14, 12, 11, 9, 10, 8};

// PWM pins on the PCB (data values) correspond to, in order:
//  Board 1 - PWM1, PWM2 | Board 2 (same), etc.
// LPC_PWM1 is port J196, LPC_PWM0 is port J197
const LPC_PWM_T * BOARD_PWM_PORTS[8] = {LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM0, LPC_PWM0, LPC_PWM0, LPC_PWM0};
const uint8_t BOARD_PWM_CHANNELS[8] = {3, 4, 5, 6, 3, 4, 5, 6};
#endif

// This can't be imported from I2CPERIPHS because it's in the .c and not the .h.
//const uint8_t ADC_Address_Select_Actuators[4] = {0x8, 0xA, 0x1A, 0x28};	// Board 0
//const uint8_t ADC_Address_Select_Actuators[4] = {0xA, 0x8, 0x1A, 0x28};	// Board 1. HACK: 1, 0, 2, 3
//const uint8_t ADC_Address_Select_Actuators[4] = {0x1A, 0x8, 0xA, 0x28};		// Board 2
const uint8_t ADC_Address_Select_Actuators[4] = {0x28, 0xA, 0x1A, 0x8};
ACTUATORS* initialize_actuator_board(uint8_t identity) {
  ACTUATORS* board = malloc(sizeof(ACTUATORS));
  board->identity = identity;
  board->bus = BOARD_I2C_BUS[board->identity];
  board->faulted = 0;

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

  // Initialize H-bridge / position fault indicators
  int fault_counter;
  for (fault_counter = 0; fault_counter < 2; fault_counter++){
      board->bridge_fault[fault_counter] = 0;
      board->pos_fault[fault_counter] = 0;

      board->consecutive_pos_faults[fault_counter] = 0;
      board->consecutive_identical_pos_faults[fault_counter] = 0;
      board->previous_invalid_pos[fault_counter] = 0;
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
  board->enable[0] = 0;     // Output disabled
  board->enable[1] = 0;     // Output disabled
  board->pwm[0] = 0;        // Stopped
  board->pwm[1] = 0;        // Stopped
  board->target_pos[0] = 0;
  board->target_pos[1] = 0;
  board->times[0][0] = 0;
  board->times[0][1] = 0;
  board->times[1][0] = 0;
  board->times[1][1] = 0;
  board->pwm_algorithm[0] = 0;
  board->pwm_algorithm[1] = 0;
  board->stalled_cycles[0] = 0;
  board->stalled_cycles[1] = 0;
  board->prev_position[0] = 0;
  board->prev_position[1] = 0;
  board->stop_mode[0] = NO_STOP;
  board->stop_mode[1] = NO_STOP;
  board->target_pwm[0] = 0.0;
  board->target_pwm[1] = 0.0;
  board->has_feedback = identity >= ACTUATOR_BOARD_BRAKING_MIN && identity <= ACTUATOR_BOARD_BRAKING_MAX;

  board->calibrated_engaged_pos[0] = -3;
  board->calibrated_engaged_pos[1] = -3;

  board->engage_step = ENGAGE_DONE;

  // Write default values to GPIO/PWM pins
  int output_counter = 0;
  for (output_counter = 0; output_counter < 2; output_counter++){
      // Direction signal
      GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + output_counter], BOARD_PINS[(board->identity * 4) + output_counter], board->direction[output_counter]);
      // Disable PWM at startup
      PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], 0.0);
  }

  return board;
}

uint8_t update_actuator_board(ACTUATORS* board) {
    // Gathers sensor data from a braking board
    // Temperature, current, and H-bridge fault signals.

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

void move_to_pos(ACTUATORS *board, int num, int destination){
	if (destination <= -3) return;

    // Begin the routine for an actuator to move to a target position

    // Do an off-cycle update to make sure that the position feedback data is not stale
    update_actuator_control(board);

    // Set the target destination value
    board->target_pos[num] = destination;
    board->prev_position[num] = board->position[num];
    board->enable[num] = 1;

    if (destination > 0){
        board->stop_mode[num] = STOP_FROM_POSITION;        // Stop based on position
        board->pwm_algorithm[num] = 1; // Exponential function based on distance to target position!
    }
    else{
        board->stop_mode[num] = NO_STOP;        // Run continuously - don't stop automatically
        board->pwm_algorithm[num] = 0; // Running continuously? Just start with the PWM-based-on-movement function
    }

    // Set a base PWM to start movement
    if (board->pwm[num] == 0){
        board->pwm[num] = MIN_DUTY_CYCLE;
    }

    if (destination == -1){
        board->direction[num] = 1;
    }
    else if (destination == -2){
        board->direction[num] = 0;
    }
    else{
        // We have a set target position, set the initial direction needed to reach it
        board->direction[num] = board->position[num] > board->target_pos[num];
    }

    if (board->direction[num] && (board->pwm[num] > MAX_FWD_DUTY_CYCLE)){
        // Constrain to max forwards duty cycle
        board->pwm[num] = MAX_FWD_DUTY_CYCLE;
    }
    else if (!board->direction[num] && (board->pwm[num] > MAX_BWD_DUTY_CYCLE)){
        // Constrain to max backwards duty cycle (higher)
        board->pwm[num] = MAX_BWD_DUTY_CYCLE;
    }


    // Calculate the target direction and duty cycle
    // Must be called here because otherwise -> enable is currently 0 and will be blocked by the guard in update_actuator_board
    // ^ Comment probably irrelevant now...
    calculate_actuator_control(board, num);

    // Then update to write the calculated control signals to the board
    update_actuator_board(board);
}

void move_to_disengaged_pos(ACTUATORS *board, int num) {
	move_to_pos(board, num, BRAKING_DISENGAGED_POSITIONS[board->identity - ACTUATOR_BOARD_BRAKING_MIN][num]);
}

void disengage_all_brakes() {
	int i, j;
	for(i=0; i<2; i++) {
		for(j=0; j<2; j++) {
			move_to_disengaged_pos(braking_boards[i], j);
		}
	}
}

void move_time(ACTUATORS *board, int num, int dir, int interval, float pwm){
    // Start moving for a set amount of time...
    board->times[num][0] = getRuntime();
    board->times[num][1] = interval;
    board->direction[num] = dir;
    board->pwm[num] = pwm;
    board->enable[num] = 1;
    board->stop_mode[num] = STOP_FROM_TIME;

}

void move_to_pwm(ACTUATORS *board, int num, int dir, float pwm){
    board->direction[num] = dir;
    board->pwm[num] = MIN_PWM_MODE_DUTY_CYCLE;
    board->target_pwm[num] = pwm;
    board->enable[num] = 1;
    board->stop_mode[num] = STOP_FROM_PWM_STALL;
    board->pwm_algorithm[num] = 0;
    board->prev_position[num] = board->position[num];
}

void calculate_actuator_control(ACTUATORS *board, int num){
    // Actuation signals and position feedback are updated here.

    // Check if actuator is even currently moving
    if (!board->enable[num] || board->pos_fault[0] || board->pos_fault[1]){
        // Actuator isn't currently moving so nothing to do here
        return;
    }

    // CHECK WHETHER TO STOP

    if (board->stop_mode[num] == STOP_FROM_TIME){
        // Check if actuator has reached time interval
        // Time control - check if time interval has been completed
        int i = 0;
        for (i = 0; i < 2; i++){
            if (board->times[i][1] > 0){
                if (getRuntime() > board->times[i][0] + board->times[i][1]){
                    // If time has elapsed, stop actuation and clear time control variable
                    board->enable[num] = 0;
                    board->times[i][1] = 0;
                    return;
                }
            }
        }
    }
    else if (!board->has_feedback){
        // Exit at this point if the board's actuators don't have feedback signals
        return;
    }
    else if (board->stop_mode[num] == STOP_FROM_POSITION){
        // Only stop based on distance if the actuator isn't set to run continuously
        if (board->target_pos >= 0){
            // Position control - Check to see if the destination has been reached (and movement should be stopped)
            if (abs(board->position[num] - board->target_pos[num]) <= 2){
                // Stop movement (and write to the PWM output immediately to prevent overshoot)
                board->enable[num] = 0;
                PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + num], BOARD_PWM_CHANNELS[(board->identity * 2) + num], 0.0);
                DEBUGOUT("Target reached! Target: %d, Actual: %d\n", board->target_pos[num], board->position[num]);
                return;
            }
         }
    }

    // UPDATE OUTPUT PWM IF STILL MOVING

    // For 'target position' mode, determine direction: forwards = 1, backwards = 0
    if (board->stop_mode[num] == STOP_FROM_POSITION){
        // Forward movement (direction = 1) => position values decrease
        board->direction[num] = board->position[num] > board->target_pos[num];
    }
    // For continuous operation in a single direction, just continue going in that direction.

    // Update the PWM based on the appropriate function
    if (board->pwm_algorithm[num] == 0){
        // PWM-based-on-feedback function (Update PWM as necessary to sustain movement)
        if (board->direction[num] && board->position[num] < board->prev_position[num]){
            board->prev_position[num] = board->position[num];
            board->stalled_cycles[num] = 0;
            board->pwm[num] -= MOVE_CYCLE_PWM_DECREASE;
            //DEBUGOUT("%d\n", board->stalled_cycles[num]);
        }
        else if (!board->direction[num] && board->position[num] > board->prev_position[num]){
            board->prev_position[num] = board->position[num];
            board->stalled_cycles[num] = 0;
            board->pwm[num] -= MOVE_CYCLE_PWM_DECREASE;
            //DEBUGOUT("%d\n", board->stalled_cycles[num]);
        }
        else{
            board->stalled_cycles[num]++;
            uint16_t required_stall_cycles = ((board->target_pwm[num] - board->pwm[num]) < 0.015 ) ? STALL_CYCLES_PWM_INCREASE * STALL_CYCLES_LAST_PWM_MULTIPLIER : STALL_CYCLES_PWM_INCREASE;
            if (board->stalled_cycles[num] >= required_stall_cycles){
                board->pwm[num] += 0.01;
                board->stalled_cycles[num] = 0;
            }
            //DEBUGOUT("%d\n", board->stalled_cycles[num]);
        }
    }
    else{
        // Exponential relation function
#if 0
        // First, determine if we're stalled and need to switch to the other algorithm
        if (board->direction[num] && board->position[num] < board->prev_position[num]){
            board->prev_position[num] = board->position[num];
            board->stalled_cycles[num] = 0;
        }
        else if (!board->direction[num] && board->position[num] > board->prev_position[num]){
            board->prev_position[num] = board->position[num];
            board->stalled_cycles[num] = 0;
        }
        else{
            board->stalled_cycles[num]++;
            if (board->stalled_cycles[num] >= STALL_CYCLES_ALG_SWITCH){
                board->stalled_cycles[num] = 0;
                board->pwm_algorithm[num] = 0;

#if PRINT_SENSOR_DATA_ACTIVE
                DEBUGOUT("Switching to variable-PWM algorithm!\n");
#endif
            }
        }
#endif

        // Otherwise, set the PWM based on the exponential function
        // Percentage of usable stroke length to travel => percentage of duty cycle to use (with minimum at 5%)
        float pct_away = (float)abs(board->position[num] - board->target_pos[num]) / USABLE_STROKE_LEN;
        float cycle = 1;
        if (pct_away < 0.5){
            // 2nd-power relation between distance to travel and output duty cycle
            cycle = 1 - pow((1 - pct_away*2), 2);
        }
        board->pwm[num] = cycle;
    }

    if(board->pwm_algorithm[num] == 0 && board->pwm[num] < MIN_PWM_MODE_DUTY_CYCLE) {
    	board->pwm[num] = MIN_PWM_MODE_DUTY_CYCLE;
    }
    else if (board->pwm_algorithm[num] == 1 && board->direction[num] == 1 && board->pwm[num] < MIN_DUTY_CYCLE) {
        board->pwm[num] = MIN_DUTY_CYCLE;
    }
    else if (board->pwm_algorithm[num] == 1 && board->direction[num] == 0 && board->pwm[num] < MIN_BWD_DUTY_CYCLE) {
        board->pwm[num] = MIN_BWD_DUTY_CYCLE;
    }
    else{
        uint8_t stop = 0;
        if (board->direction[num] && (board->pwm[num] > MAX_FWD_DUTY_CYCLE)){
            // Constrain to max forwards duty cycle
            board->pwm[num] = MAX_FWD_DUTY_CYCLE;
            // Only stop if using the pwm-incrementation algorithm
            stop = (board->pwm_algorithm == 0);
        }
        else if (!board->direction[num] && (board->pwm[num] > MAX_BWD_DUTY_CYCLE)){
            // Constrain to max backwards duty cycle (higher)
            board->pwm[num] = MAX_BWD_DUTY_CYCLE;
            // Only stop if using the pwm-incrementation algorithm
            stop = (board->pwm_algorithm == 0);
        }
        else if (board->stop_mode[num] == STOP_FROM_PWM_STALL){
            // Stop based on the PWM limit that was set
            if (board->pwm[num] >= board->target_pwm[num]){
            	stop = 1;
                //stop = (board->pwm_algorithm == 0);
            }
        }
        if (stop){
            board->enable[num] = 0;
#if PRINT_SENSOR_DATA_ACTIVE
            DEBUGOUT("Actuator #%d PWM has reached max limit %f in %s direction, and movement has been stopped! \n", num, board->pwm[num], (board->direction[num] ? "forwards" : "backwards"));
#endif
        }
    }

#if PRINT_SENSOR_DATA_ACTIVE
    //DEBUGOUT("Output duty cycle is %f \n", board->pwm[num]);
#endif

}

void update_actuator_control(ACTUATORS *board){
    // Update data relevant to braking actuator movement
    //  This function is separated to allow for fast sampling of position feedback
    // Read position feedback signal from linear potentiometer -> ADC
    // Write direction signal to GPIO pins, set output PWM frequency

    int i = 0;
    for (i = 0; i < 2; i++){
        // Read and process position feedback if present on the board/actuators.
        if (board->has_feedback){
            uint16_t pos = ADC_read(board->bus, board->ADC_device_address, 5 - (4 * i));

            //  Calculate a moving average using only those values not considered erroneous
            if ((abs(board->position[i] - pos) < POS_MAX_DELTA) || (board->stop_mode[i] == STOP_FROM_PWM_STALL)){
                // Only include values within the tolerance window POS_MAX_DELTA, unless the go-until-stalled-at-X%-pwm control scheme is in use.
                board->position[i] = (POS_MOV_AVG_ALPHA * pos) + ((1 - POS_MOV_AVG_ALPHA) * board->position[i]);
            	board->consecutive_pos_faults[i] = 0;
        	    board->consecutive_identical_pos_faults[i] = 0;
            }
            else{
                DEBUGOUT("%d[%d]: Throwing out value [%d] while at position [%d]\n", board->identity, i, pos, board->position[i]);
            	board->consecutive_pos_faults[i]++;
            	DEBUGOUT("Pos: %d | Prev Pos: %d | ABS: %d\n", pos, board->previous_invalid_pos[i], abs(pos - board->previous_invalid_pos[i]));
            	if(abs(pos - board->previous_invalid_pos[i]) < 25 /*FIXME*/) {
            	    board->consecutive_identical_pos_faults[i]++;
            	}
            	else {
            	    board->consecutive_identical_pos_faults[i] = 0;
            	}
            	DEBUGOUT("Consecutive pos faults: %d | Consecutive identical pos faults: %d\n",
            		board->consecutive_pos_faults[i], board->consecutive_identical_pos_faults[i]);

                board->previous_invalid_pos[i] = pos;

                if(board->consecutive_pos_faults[i] == 3) {
                	DEBUGOUT("ERROR: Too many consecutive position faults! Setting pos_fault high!\n");
                	board->pos_fault[i] = 1;
                }
                if(board->consecutive_identical_pos_faults[i] == 3) {
                	DEBUGOUT("NOTICE: Consecutive identical position faults indicate that stored position is incorrect! Setting pos_fault low!\n");
                	board->pos_fault[i] = 0;

                	// Set position equal to "fault" position.
                	board->position[i] = pos;

                	// Reset fault counters.
                	board->consecutive_pos_faults[i] = 0;
                	board->consecutive_identical_pos_faults[i] = 0;
                }
            }
        }

        // Service actuator movement routine if it is currently active
        if (board->enable[i]){
            calculate_actuator_control(board, i);
        }
    }

    // Update direction and PWM
    int output_counter = 0;
    for (output_counter = 0; output_counter < 2; output_counter++){
        // Direction signal
        GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + output_counter], BOARD_PINS[(board->identity * 4) + output_counter], board->direction[output_counter]);
        // PMW enable/speed signal
        if (board->enable[output_counter] && !(board->pos_fault[0] || board->pos_fault[1])){
            PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], board->pwm[output_counter]);
        }
        else{
            PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + output_counter], BOARD_PWM_CHANNELS[(board->identity * 2) + output_counter], 0.0);
        }
    }
}

void start_actuator_calibration() {
	calibration_step = CALIBRATION_INIT;
}

void update_actuator_calibration(ACTUATORS *board) {
	int i = 0;
	int stopped = (board->enable[0] == 0) /*&& (board->enable[1] == 0)*/;

	switch(calibration_step) {
	case CALIBRATION_DONE:
		// Nothing to do here.
		break;
	case CALIBRATION_INIT:
		if(stopped) {
			DEBUGOUT("CALIBRATION_INIT\n");
			start_actuator_disengage(board);
			calibration_step++;
		}
		break;
	case CALIBRATION_APPROACH:
		if(stopped) {
			DEBUGOUT("CALIBRATION_APPROACH\n");
			//for(i=0; i<2; i++) {
				move_to_pos(board, i, board->position[i] - 600);
			//}
			calibration_step++;
		}
		break;
	case CALIBRATION_MOVE_TO_PWM:
		if(stopped) {
			DEBUGOUT("CALIBRATION_MOVE_TO_PWM\n");
			//for(i=0; i<2; i++) {
				move_to_pwm(board, i, OUT, MIN_DUTY_CYCLE);
			//}
			calibration_step++;
		}
		break;
	case CALIBRATION_DEINIT:
		if(stopped) {
			DEBUGOUT("CALIBRATION_DEINIT\n");
			calibration_step++;
			//for(i=0; i<2; i++) {
				board->calibrated_engaged_pos[i] = board->position[i];
			//}
			start_actuator_ready(board);
			calibration_step = CALIBRATION_DONE;
		}
		break;
	}
}

void start_actuator_engage(ACTUATORS *board) {
	if(board->calibrated_engaged_pos[0] < 0 /*|| board->calibrated_engaged_pos[1] < 0*/) {	// FIXME
		return;
	}

#if 1
	board->engage_step = ENGAGE_MOVE_TO_CHECKPOINT;
#else
	for(i=0; i<2; i++) {
		move_to_pos(board, i, board->calibrated_engaged_pos[0 /* i */]);	// FIXME
	}
#endif
}

void update_actuator_engage(ACTUATORS *board) {
	int i = 0;
	int stopped = (board->enable[0] == 0) && (board->enable[1] == 0);

	switch(board->engage_step) {
	case ENGAGE_DONE:
		// Nothing to do here.
		break;
	case ENGAGE_MOVE_TO_CHECKPOINT:
		if(stopped) {
			DEBUGOUT("ENGAGE_MOVE_TO_CHECKPOINT\n");
			for(i=0; i<2; i++) {
				start_actuator_ready(board);
			}
			board->engage_step++;
		}
		break;
	case ENGAGE_MOVE_TO_TARGET:
		if(stopped) {
			DEBUGOUT("ENGAGE_MOVE_TO_TARGET\n");
			for(i=0; i<2; i++) {
				move_to_pos(board, i, board->calibrated_engaged_pos[0 /* i */]);	// FIXME
			}
			board->engage_step = ENGAGE_DONE;
		}
		break;
	}
}

void start_actuator_ready(ACTUATORS *board) {
	if(board->calibrated_engaged_pos[0] < 0 /*|| board->calibrated_engaged_pos[1] < 0*/) {	// FIXME
		return;
	}

	int i=0;
	for(i=0; i<2; i++) {
		move_to_pos(board, i, board->calibrated_engaged_pos[0 /* i */] + 250);
	}
}

void start_actuator_disengage(ACTUATORS *board) {
	int i=0;
	for(i=0; i<2; i++) {
		move_to_pos(board, i, BRAKING_DISENGAGED_POSITIONS[board->identity][i]);
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
