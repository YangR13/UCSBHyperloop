#include "actuators.h"
#include "pwm.h"

const uint8_t BOARD_I2C_BUS[NUM_ACTUATOR_BOARDS] = {1, 0, 0, 0}; // TODO: Set me
const uint8_t BOARD_I2C_DIP[NUM_ACTUATOR_BOARDS] = {0,  255,  0,  255}; // TODO: Change to realistic values!
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
const uint8_t BOARD_PINS[12] = {DIR1, DIR2, FAULT1, FAULT2, PWM1, PWM2};
const uint8_t ARDUINO_ADC_PINS = {POS1, POS2, CUR1, CUR2, T1, T2, T3, T4};
#endif
#ifdef LPC


// TODO: Change these to realistic pin values!
// Board pins, in order, are
// Board 1 - DIR1, DIR2, FAULT1, FAULT2 | Board 2, etc
const uint8_t BOARD_PIN_PORTS[16] = {0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t BOARD_PINS[16] =      {0, 12, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const LPC_PWM_T * BOARD_PWM_PORTS[8] = {LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1, LPC_PWM1};
const uint8_t BOARD_PWM_CHANNELS[8] = {0, 1, 0, 0, 0, 0, 0, 0};
/* Alternate arrangement (in order of ADC channel)
const uint8_t ADC_PORTS[8] = {LTC2309_CHN_0, LTC2309_CHN_1, LTC2309_CHN_2, LTC2309_CHN_3, LTC2309_CHN_4, LTC2309_CHN_5, LTC2309_CHN_6, LTC2309_CHN_7};
// ADC ports correspond to: Temp4, Pos2, Temp3, Curr2, Temp2, Pos1, Temp1, Curr1
*/

const uint8_t ADC_PORTS[8] = {LTC2309_CHN_6, LTC2309_CHN_4, LTC2309_CHN_2, LTC2309_CHN_0, LTC2309_CHN_5, LTC2309_CHN_1, LTC2309_CHN_7, LTC2309_CHN_3};
// ADC_PORTS correspond to: Temp1, Temp2, Temp3, Temp4, Pos1, Pos2, Curr1, Curr2
#endif

const uint8_t ADC_Address_Select[4] = {0x8, 0xA, 0x1A, 0x28};


ACTUATORS* initialize_actuator_board(uint8_t identity) {
  ACTUATORS* board = malloc(sizeof(ACTUATORS));
  board->identity = identity;
  board->bus = BOARD_I2C_BUS[board->identity];
  board->ADC_device_address[0] = ADC_Address_Select[(BOARD_I2C_DIP[board->identity] >> 6) & 0b11];

  // Initialize thermistor moving averages (with first read)
  int temp_counter;
  for (temp_counter = 0; temp_counter < 4; temp_counter++) {
    board->temperatures[temp_counter] = calculate_temperature(ADC_read_actuators(board->bus, board->ADC_device_address[0], ADC_PORTS[temp_counter]));
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

  // Initialize GPIO/PWM pins
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 0], BOARD_PINS[(board->identity * 4) + 0], OUT); // Direction 1
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 1], BOARD_PINS[(board->identity * 4) + 1], OUT); // Direction 2
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 2], BOARD_PINS[(board->identity * 4) + 2], IN); // H-bridge 1 fault
  GPIO_Setup(BOARD_PIN_PORTS[(board->identity * 4) + 3], BOARD_PINS[(board->identity * 4) + 3], IN); // H-bridge 2 fault
  PWM_Setup(BOARD_PWM_PORTS[(board->identity * 2) + 0], BOARD_PWM_CHANNELS[(board->identity * 2) + 0]); // PWM output 1
  PWM_Setup(BOARD_PWM_PORTS[(board->identity * 2) + 1], BOARD_PWM_CHANNELS[(board->identity * 2) + 1]); // PWM output 2

  return board;
}


uint8_t update_actuator_board(ACTUATORS* board) {
  //Record Temperatures
  int temp_counter;
  for (temp_counter = 0; temp_counter < NUM_THERMISTORS; temp_counter++) {
    int new_temperature = calculate_temperature(ADC_read_actuators(board->bus, board->ADC_device_address, ADC_PORTS[temp_counter]));
    new_temperature = ((1 - THERMISTOR_AVG_WEIGHT) * new_temperature + THERMISTOR_AVG_WEIGHT * board->temperatures[temp_counter]);
    board->temperatures[temp_counter] = new_temperature;
    /*
    if (new_temperature > HEMS_MAX_TEMP || new_temperature < HEMS_MIN_TEMP){
      board->alarm |= 0b00000001;
    }
    */
  }

  /*
  // Record Motor Controller Currents
  // With no current, the ACS759x150B should output 3.3V/2
  int amps_counter;
  for (amps_counter = 0; amps_counter < 2; amps_counter++){
      uint16_t ammeter_ratio = ADC_read(board->bus, board->ADC_device_address, 2 + amps_counter);
      uint8_t new_amps = abs(1000 * ammeter_ratio * HEMS_CAL_5V0REF[board->identity] / MAX12BITVAL - 1000 * HEMS_CAL_3V3REF[board->identity] / 2) / AMMETER_150A_SENSITIVITY; //Done in mV
      board->amps[amps_counter] = new_amps;

//      if (new_amps > HEMS_MAX_CURRENT){
//          board->alarm |= 0b00000010;
//      }
  }
  */

  // Update direction and PWM
  GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + 0], BOARD_PINS[(board->identity * 4) + 0], board->direction[0]); // Direction 1
  GPIO_Write(BOARD_PIN_PORTS[(board->identity * 4) + 1], BOARD_PINS[(board->identity * 4) + 1], board->direction[1]); // Direction 2
  PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + 0], BOARD_PWM_CHANNELS[(board->identity * 2) + 0], board->enable[0]); // PWM output 1
  PWM_Write(BOARD_PWM_PORTS[(board->identity * 2) + 1], BOARD_PWM_CHANNELS[(board->identity * 2) + 1], board->enable[1]); // PWM output 2

  // Read fault flags
  board->bridge_fault[0] = GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 2], BOARD_PINS[(board->identity * 4) + 2]);
  board->bridge_fault[1] = GPIO_Read(BOARD_PIN_PORTS[(board->identity * 4) + 3], BOARD_PINS[(board->identity * 4) + 3]);

  //return board->alarm;
  return 0;
}

/*
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
*/


void PWM_Setup(const void * pwm, uint8_t pin){
#ifdef ARDUINO
    // Do arduino things.
    // set as output.
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

uint16_t ADC_read_actuators(uint8_t bus, uint8_t ADC_address, uint8_t ADC_channel) {
  uint8_t input_buffer[2];

#ifdef ARDUINO
  Wire.beginTransmission(ADC_address);
  Wire.write(ADC_CHANNEL_SELECT[ADC_channel]);
  Wire.endTransmission(true);
  Wire.requestFrom(ADC_address, 2, true);
  input_buffer[0] = Wire.read();  //D11 D10 D9 D8 D7 D6 D5 D4
  input_buffer[1] = Wire.read();  //D3 D2 D1 D0 X X X X
#endif //ARDUINO
#ifdef LPC
  Chip_I2C_MasterSend(bus, ADC_address, ADC_channel | ADC_CONFIG, 1);
  Chip_I2C_MasterRead(bus, ADC_address, input_buffer, 2);
#endif //LPC

  uint16_t ADC_value = (input_buffer[0] << 4) | (input_buffer[1] >> 4);
  return ADC_value;
}

#if 0

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
#endif

