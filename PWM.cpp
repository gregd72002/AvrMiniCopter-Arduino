/* 
FAST_PWM implementation is based on MultiWii Output implementation:
https://github.com/multiwii/multiwii-firmware/blob/upstream_shared/Output.cpp
*/

#include "Arduino.h"

int16_t motor[4];

uint8_t PWM_PIN[4] = {9,3,6,5};

#define FAST_PWM 1

#ifdef FAST_PWM
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;

void writeMotors() { // [1000;2000] => [125;250]
	OCR1A = motor[0]>>3; //  pin 9
	OCR2B = motor[1]>>3; //  pin 3
	atomicPWM_PIN6_highState = motor[2]>>3;
	atomicPWM_PIN5_highState = motor[3]>>3;
	atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
	atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState; 
}

void initPWM() {
	for(uint8_t i=0;i<4;i++) {
		pinMode(PWM_PIN[i],OUTPUT);
	}

	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
	TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B

	TCCR0A = 0; // normal counting mode
	TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
	TIMSK0 |= (1<<OCIE0A);
}


#define SOFT_PWM_ISR1 TIMER0_COMPB_vect
#define SOFT_PWM_ISR2 TIMER0_COMPA_vect
#define SOFT_PWM_CHANNEL1 OCR0B
#define SOFT_PWM_CHANNEL2 OCR0A 

#define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
#define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
#define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
#define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);

ISR(SOFT_PWM_ISR1) { 
	static uint8_t state = 0;
	if(state == 0){
		if (atomicPWM_PIN5_highState>0) SOFT_PWM_1_PIN_HIGH;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_1_PIN_LOW;
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
		state = 3;  
	}else if(state == 3){
		SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
		state = 0;   
	}
}
ISR(SOFT_PWM_ISR2) { 
	static uint8_t state = 0;
	if(state == 0){
		if (atomicPWM_PIN6_highState>0) SOFT_PWM_2_PIN_HIGH;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
		state = 1;
	}else if(state == 1){
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
		state = 2;
	}else if(state == 2){
		SOFT_PWM_2_PIN_LOW;
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
		state = 3;  
	}else if(state == 3){
		SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
		state = 0;   
	}
}

#else

#include <Servo.h>
Servo myservo[4];

void writeMotors() {
	for (int8_t i=0;i<4;i++)
		myservo[i].writeMicroseconds(motor[i]);
}

void initPWM() {
      for (int8_t i=0;i<4;i++)
		myservo[i].attach(PWM_PIN[i]);
}

#endif


