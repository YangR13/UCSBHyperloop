#include "braking_actuators.h"


#define DIR1 52
#define PWM1 2
#define DIR2 53
#define PWM2 3
#define POS1 A0
#define CUR1 A1
#define POS2 A2
#define CUR2 A3


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

