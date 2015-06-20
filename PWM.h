#ifndef PWM_H_
#define PWM_H_

extern uint8_t PWM_PIN[4];
extern int16_t motor[4];

void initPWM();
void deinitPWM();
void writeMotors();

#endif /* PWM_H_ */
