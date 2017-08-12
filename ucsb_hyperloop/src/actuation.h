#ifndef actuation_h
#define actuation_h

#define MAX_THROTTLE_VOLTAGE 5 //[V]

#define PAYLOAD_MOVE_TIME 3000 // milliseconds? Check units of getRuntime()
#define SERV_MOTOR_ACT_MOVE_TIME 3000 // milliseconds? ^^^
#define SERV_MOTOR_DUTY 0.50 // Percentage duty cycle

void performActuation();
void set_motor_throttle(int motor_num, float voltage);

void actuate_brakes();
void actuate_maglev();
void actuate_payload();
void actuate_service();

#endif
