#ifndef PWM_H_
#define PWM_H_

extern uint8_t PWM_PIN[8];
extern uint16_t motor[8];

void initPWM();
void writeMotors();

#endif /* PWM_H_ */
