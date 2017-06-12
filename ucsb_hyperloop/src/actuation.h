#ifndef actuation_h
#define actuation_h

#define MAX_THROTTLE_VOLTAGE 5 //[V]

void performActuation();
void set_motor_throttle(int motor_num, float voltage);

void actuate_brakes();
void actuate_maglev();
void init_maglev_test();
void actuate_maglev_test();
void actuate_payload();
void actuate_service();

#endif
