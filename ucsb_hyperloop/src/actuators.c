#include "actuators.h"

const uint8_t BOARD_I2C_BUS[NUM_ACTUATOR_BOARDS] = {0, 0, 0, 0};
const uint8_t ACTUATOR_I2C_DIP[NUM_ACTUATOR_BOARDS] = {0,  255,  0,  255}; // TODO: Change to realistic values!
#ifdef ARDUINO
// Outputs
#define DIR1 52
#define PWM1 2
#define DIR2 53
#define PWM2 3
// Inputs
#define POS1 A0
#define CUR1 A1
#define POS2 A2
#define CUR2 A3
// TODO: Change these to realistic pin values!
#define T1 0
#define T2 0
#define T3 0
#define T4 0
#define FAULT1 0
#define FAULT2 0
const uint8_t BOARD_PIN_PORTS[4] = {0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PINS[12] = {DIR1, PWM1, DIR2, PWM2, FAULT1, FAULT2};
const uint8_t ARDUINO_ADC_PINS = {POS1, CUR1, POS2, CUR2, T1, T2, T3, T4};
#endif
#ifdef LPC
// TODO: Change these to realistic pin values!
const uint8_t BOARD_PIN_PORTS[4] = {0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PINS[12] = {0, 0, 0, 0, 0, 0, 0};
const uint8_t ADC_PORTS = {LTC2309_CHN_0, LTC2309_CHN_1, LTC2309_CHN_2, LTC2309_CHN_3, LTC2309_CHN_4, LTC2309_CHN_5, LTC2309_CHN_6, LTC2309_CHN_7};
#endif



ACTUATORS* initialize_actuator_board(uint8_t identity) {
  ACTUATORS* board = malloc(sizeof(ACTUATORS));
  board->identity = identity;
  board->bus = BOARD_I2C_BUS[board->identity];
  board->ADC_device_address = ADC_Address_Select[(HEMS_I2C_DIP[board->identity] >> 6) & 0b11];

  // Initialize thermistor moving averages (with first read)
  int temp_counter;
  for (temp_counter = 0; temp_counter < 4; temp_counter++) {
    board->temperatures[temp_counter] = calculate_temperature(ADC_read(board->bus, board->ADC_device_address[0], temp_counter + 1));
  }

  board->amps = 0;
  board->alarm = 0;

  GPIO_Setup(uint8_t port, uint8_t pin, uint8_t dir); // Direction
  GPIO_Setup(uint8_t port, uint8_t pin, uint8_t dir); // H-bridge 1 fault
  GPIO_Setup(uint8_t port, uint8_t pin, uint8_t dir); // H-bridge 2 fault
  PWM_Setup(uint8_t port, uint8_t pin, uint8_t dir); // PWM output


  return board;
}


uint8_t update_actuator_board(ACTUATORS* board) {
  //Record Temperatures
  int temp_counter;
  for (temp_counter = 0; temp_counter < 4; temp_counter++) {
    int new_temperature = calculate_temperature(ADC_read(board->bus, board->ADC_device_address, temp_counter + 1));
    new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * board->temperatures[temp_counter]);
    board->temperatures[temp_counter] = new_temperature;
    if (new_temperature > HEMS_MAX_TEMP || new_temperature < HEMS_MIN_TEMP)
      board->alarm |= 0b00000001;
  }

  // Record Motor Controller Current
  // With no current, the ACS759x150B should output 3.3V/2
  uint16_t ammeter_ratio = ADC_read(board->bus, board->ADC_device_address, 7);
  uint8_t new_amps = abs(1000 * ammeter_ratio * HEMS_CAL_5V0REF[board->identity] / MAX12BITVAL - 1000 * HEMS_CAL_3V3REF[board->identity] / 2) / AMMETER_150A_SENSITIVITY; //Done in mV
  board->amps = new_amps;

  if (new_amps > HEMS_MAX_CURRENT)
    board->alarm |= 0b00000010;

  return board->alarm;
}

void GPIO_Setup(uint8_t port, uint8_t pin, uint8_t dir) {
#ifdef ARDUINO
  if (dir == IN) pinMode(AUX_PIN, INPUT);
  else if (dir == OUT) pinMode(AUX_PIN, OUTPUT);
#endif //ARDUINO
#ifdef LPC
  if (dir == IN) GPIO_Input_Init(port, pin);
  else if (dir == OUT) GPIO_Output_Init(port, pin);
#endif //LPC
}

void GPIO_Write(uint8_t port, uint8_t pin, uint8_t setting) {
#ifdef ARDUINO
  digitalWrite(AUX_PIN, setting);
#endif //ARDUINO
#ifdef LPC
  Chip_GPIO_SetPinState(LPC_GPIO, port, pin, setting);
#endif //LPC
}

uint8_t GPIO_Read(uint8_t port, uint8_t pin) {
#ifdef ARDUINO
  return digitalRead(AUX_PIN);
#endif //ARDUINO
#ifdef LPC
  return Chip_GPIO_GetPinState(LPC_GPIO, port, pin);
#endif //LPC
}

void PWM_Write(uint8_t port, uint8_t pin, uint8_t setting) {
#ifdef ARDUINO
    analogWrite(AUX_PIN, setting);
#endif // ARDUINO
#ifdef LPC
    // WRITE TO PWM HERE
#endif // LPC
}




#define PWM_CYCLE 253
#define ALPHA 0.5
#define WINDOW 5

//initialize variables
int target = 0;
int is_adjusting_1 = 0;
int location_1 = 0;
int feedback_1 = 0;
int is_adjusting_2 = 0;
int location_2 = 0;
int feedback_2 = 0;

void setup() {

  //enable output pins
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  //begin serial communication
  Serial.begin(9600);

}

void loop() {

  if(Serial.available() > 0) { //input available

    //read input, set as new target
    target = Serial.parseInt();
    if(target < 17) target = 17;
    if(target > 555) target = 555;
    Serial.println();
    Serial.println(target);
    Serial.println();
    is_adjusting_1 = 1;
    is_adjusting_2 = 1;

  }

  if(is_adjusting_1 == 1) { //evaluate behavior

    //read feedback
    feedback_1 = analogRead(POS1);
    //Serial.println(feedback);

    //update location, rolling average filter
    location_1 = (1-ALPHA)*location_1 + ALPHA*feedback_1;
    Serial.print(feedback_1);

    //determine direction of motion
    if(((target - WINDOW) < location_1) && (location_1 < (target + WINDOW))) { //at correct location

      //disable output
      analogWrite(PWM1,0);
      is_adjusting_1 = 0;

    }

    else {
      //Serial.println(location);
      if(location_1 <= (target - WINDOW)) { //has to advance

        //advance
        digitalWrite(DIR1, HIGH);
        analogWrite(PWM1, PWM_CYCLE);

      }

      else { //has to retract

        //retract
        digitalWrite(DIR1, LOW);
        analogWrite(PWM1, PWM_CYCLE);

      }
    }
  }

    if(is_adjusting_2 == 1) { //evaluate behavior

    //read feedback
    feedback_2 = analogRead(POS2);
    //Serial.println(feedback);

    //update location, rolling average filter
    location_2 = (1-ALPHA)*location_2 + ALPHA*feedback_2;
    Serial.print('\t');
    Serial.print(location_2);

    //determine direction of motion
    if(((target - WINDOW) < location_2) && (location_2 < (target + WINDOW))) { //at correct location

      //disable output
      analogWrite(PWM2,0);
      is_adjusting_2 = 0;

    }

    else {
      if(location_2 <= (target - WINDOW)) { //has to advance

        //advance
        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, PWM_CYCLE);

      }

      else { //has to retract

        //retract
        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, PWM_CYCLE);

      }
    }
  }
  Serial.println();
}

