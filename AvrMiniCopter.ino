/*
   Copyright 2015, Gregory Dymarek  gregd72002@gmail.com
 */


#include "Arduino.h"
#ifdef DEBUG
#include "freeram.h"
#endif


uint8_t crc_err;
uint8_t mpu_err;
#include "mpu.h"
#include "I2Cdev.h"
#include <SPI.h>
#include "SPIdev.h"
#include "crc8.h"
#include "pid.h"
#include "buf.h"
//#include <Servo.h>
#include "PWM.h"

int ret;
#define SERVO_FL 0
#define SERVO_BL 1
#define SERVO_FR 2
#define SERVO_BR 3

byte motor_order = 228;

/*
   We are using the following pins to control ESCs: 9, 3, 6, 5

   Motor_order is a 4x 2bit value for the following motors: FL, BL, FR, BR 
   Each 2bit value defines the PIN:

   Value DEC:	 0  1  2  3
   Value BIN:      00 01 10 11
Pin:             9  3  6  5
 */

uint8_t mpu_addr; 
int16_t motor_pwm[4]; //standby, min, inflight threshold, hoover

struct s_pid pid_r[3];
struct s_pid pid_s[3];
float pid_acro_p;

uint8_t loop_ms = 5;
float loop_s = 0.005f;
unsigned long p_millis = 0;

#define YAW_THRESHOLD 8
#define MAX_ALT 20000 //200m (ensure MAX_ALT + MAX_ALT_INC fits into signed int)
#define MAX_ALT_INC 1000 //10m
#define MAX_ACCEL  250
struct s_pid pid_accel, pid_alt, pid_vz;

struct s_buf<float> alt_buf;

float ld;
float accel_z = 0.f;

float accel_err = 0.f;
float accel_corr = 0.f;
float alt_corr = 0.f;
float vz_inc = 0.f;
float vz_err = 0.f;
float alt_err = 0.f;
float alt_base = 0.f;

float alt = 0.f;
float vz = 0.f;
float pos_err = 0.f, vz_target = 0.f, vz_desired;

float bc=0.f,bc1=0.f,bc2=0.f,bc3=0.f;
uint8_t baro_counter = 0;

int8_t alt_hold = 0;
float alt_hold_target;

int8_t failsafe = 0;
unsigned long max_failsafe_ms = 30000;
unsigned long failsafeStart = 0u;
float accel_crash_detector = 0.f;
unsigned long last_command = 0; 

uint8_t loop_count = 0u;
#ifdef DEBUG
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
#endif


uint8_t status = 0;
int8_t code = 0;
uint8_t fly_mode;
uint8_t log_mode;


uint8_t i;

//for identity matrix see inv_mpu documentation how this is calculated; this is overwritten by a config packet
uint8_t gyro_orientation = 136;
float yaw_target;
byte packet[4];

int yprt[4] = {0,0,0,0};

void motor_idle() {
	alt_hold = 0;
	yprt[3] = motor_pwm[0];
	for (i=0;i<4;i++)
		motor[i] = motor_pwm[0];

	writeMotors();
}

void initMotors() {
	initPWM();
	delay(300);
	motor_idle();
}

void deinitMotors() {
	deinitPWM();
}

void sendPacket(byte t, int v) {
	packet[0] = t;
	packet[1] = v & 0x00FF;
	packet[2] = (v & 0xFF00) >> 8;
	SPI_sendBytes(packet,3);
}

void initAVR() {
	for (i=0;i<3;i++) {
		pid_init(&pid_r[i]);
		pid_init(&pid_s[i]);
	}
#ifdef ALTHOLD
	pid_init(&pid_accel);
	pid_init(&pid_vz);
	pid_init(&pid_alt);
	buf_init(&alt_buf,15);
#endif
	bc = 6.f;
	bc1 = 0.5f;
	bc2 = 0.833333f;
	bc3 = 0.004629f;

	yaw_target = 0.f;
	loop_count = 0;
	status = 0;
	fly_mode = 0;
	log_mode = 0;

	mpu_err = 0;
	alt_hold = 0;
	alt_hold_target = 0.f;
	accel_z = 0.f;

	accel_err = 0.f;
	accel_corr = 0.f;
	alt_corr = 0.f;
	vz_inc = 0.f;
	alt_err = 0.f;
	alt_base = 0.f;

	alt = 0.f;
	vz = 0.f;
	
	SPI_reset();
}

void setup() {
	Fastwire::setup(400,0);

#ifdef DEBUG
	Serial.begin(115200);
#endif
	pinMode(MISO,OUTPUT);
	SPCR |= _BV(SPE);
	//SPI.setDataMode(SPI_MODE0);
	//SPI.setBitOrder(MSBFIRST);
	//SPI.attachInterrupt(); //alternatively:
	SPCR |= _BV(SPIE);
#ifdef DEBUG
	Serial.print("MPU init: "); Serial.println(ret);
	Serial.print("Free mem: "); Serial.println(freeRam());
#endif
}

void sendStatus() {
  sendPacket(255, (status<<8) | uint8_t(code) );
}

void process_command() { 

	byte t = 0;
	int v = 0;

	unsigned long t_lag = (millis() - last_command);
	if ((status==5) && (!failsafe) && (yprt[3]>motor_pwm[1])) { //dont check for lag if not spinning 
		if (t_lag>2500) { 
			code = 3;
			failsafe = 1;
		} else if (t_lag>500) {
			yprt[0] = yprt[1] = yprt[2] = 0;
			fly_mode = 0;
		} 
	}

	//each command is 4 byte long: what, value(2), crc - do it till buffer empty

	if (SPI_getPacket(packet)==0) {
		t = packet[0];
		v = packet[2] << 8 | packet[1];
		switch(t) {
#ifdef DEBUG
			case 0: Serial.println("Received packet 0! Check your SPI connection!"); //this usually happens by default when SPI wiring is not right
				break;
#endif
			case 2: log_mode = v; 
				break;
			case 3: fly_mode = v; 
				yaw_target = mympu.ypr[0];    //when changing fly_mode during flight reset the yaw_target                                           
				break;
			case 4: 
				gyro_orientation = v;
				break;

			case 5: 
				motor_order = v;
				break;

			case 6: 
				initMotors();
				break;

			case 9: mpu_addr = v; break;
			case 10: yprt[0] = v; break;
			case 11: yprt[1] = v; break;
			case 12: yprt[2] = v; break;
			case 13: last_command = millis(); yprt[3] = v; break;
#ifdef ALTHOLD
			case 14: //altitude reading in cm - convert it into altitude error 
				 if (alt_hold && abs(alt-v)>1000) return;  //baro is glitching or we are in a free fall
				 static float b;
				 baro_counter = 200; //use this baro reading not more than 200 times (this is 1000ms as the loop goes with 200Hz - 5ms)
				 if (!buf_space(&alt_buf)) b = buf_pop(&alt_buf); //buffer full
				 else b = alt_base;
				 alt_err = v - (b + alt_corr);
				 break;
			case 15:   alt_hold = v; 
				   alt_hold_target = alt;
				   break;
			case 16: 
				   if (failsafe) return;
				   if (v>MAX_ALT_INC) alt_hold_target += MAX_ALT_INC; 
				   else if (v<-MAX_ALT_INC) alt_hold_target -= MAX_ALT_INC;
				   else alt_hold_target += v;
				   if (alt_hold_target>MAX_ALT) alt_hold_target=MAX_ALT;
				   break;
#endif
			case 17: motor_pwm[0] = v; break;
			case 18: motor_pwm[1] = v; break;
			case 19: motor_pwm[3] = v; break;
			case 20: max_failsafe_ms = abs(v)*1000; break;
			case 21: accel_crash_detector = (float)v/100.f; break;
			case 22: motor_pwm[2] = v; break;

			case 25: 
				 if (!failsafe) failsafe = 1; 
				 break;

			case 69: 
				 bc = v/100.f;
				 bc1 = 3.f/(bc);
				 bc2 = 3.f/(bc*bc);
				 bc3 = 1.f/(bc*bc*bc);
				 break;
			case 70: pid_accel.max = v; break; 
			case 71: pid_accel.imax = v; break; 
			case 72: pid_accel.Kp = (float)v/1000.f; break; 
			case 73: pid_accel.Ki = (float)v/1000.f; break; 
			case 74: pid_accel.Kd = (float)v/10000.f; break; 
			case 80: pid_alt.max = v; break; 
			case 81: pid_alt.imax = v; break; 
			case 82: pid_alt.Kp = (float)v/1000.f; break; 
			case 83: pid_alt.Ki = (float)v/1000.f; break; 
			case 84: pid_alt.Kd = (float)v/10000.f; break; 
			case 90: pid_vz.max = v; break; 
			case 91: pid_vz.imax = v; break; 
			case 92: pid_vz.Kp = (float)v/1000.f; break; 
			case 93: pid_vz.Ki = (float)v/1000.f; break; 
			case 94: pid_vz.Kd = (float)v/10000.f; break; 

			case 100: pid_r[0].max = v; break; 
			case 101: pid_r[0].imax = v; break; 
			case 102: pid_r[0].Kp = (float)v/1000.f; break; 
			case 103: pid_r[0].Ki = (float)v/1000.f; break; 
			case 104: pid_r[0].Kd = (float)v/10000.f; break; 
			case 110: pid_r[1].max = v; break; 
			case 111: pid_r[1].imax = v; break; 
			case 112: pid_r[1].Kp = (float)v/1000.f; break; 
			case 113: pid_r[1].Ki = (float)v/1000.f; break; 
			case 114: pid_r[1].Kd = (float)v/10000.f; break; 
			case 120: pid_r[2].max = v; break; 
			case 121: pid_r[2].imax = v; break; 
			case 122: pid_r[2].Kp = (float)v/1000.f; break; 
			case 123: pid_r[2].Ki = (float)v/1000.f; break; 
			case 124: pid_r[2].Kd = (float)v/10000.f; break; 

			case 130: pid_acro_p = v/1000.f; break; 

			case 200: pid_s[0].max = v; break; 
			case 201: pid_s[0].imax = v; break; 
			case 202: pid_s[0].Kp = (float)v/1000.f; break; 
			case 203: pid_s[0].Ki = (float)v/1000.f; break; 
			case 204: pid_s[0].Kd = (float)v/10000.f; break; 
			case 210: pid_s[1].max = v; break; 
			case 211: pid_s[1].imax = v; break; 
			case 212: pid_s[1].Kp = (float)v/1000.f; break; 
			case 213: pid_s[1].Ki = (float)v/1000.f; break; 
			case 214: pid_s[1].Kd = (float)v/10000.f; break; 
			case 220: pid_s[2].max = v; break; 
			case 221: pid_s[2].imax = v; break; 
			case 222: pid_s[2].Kp = (float)v/1000.f; break; 
			case 223: pid_s[2].Ki = (float)v/1000.f; break; 
			case 224: pid_s[2].Kd = (float)v/10000.f; break; 
			case 250: case 251: case 252: case 253:
				  motor[(motor_order >> (2*(t-250))) & 0x3] = v;
				  writeMotors();
				  break;
			case 244: status = v; break;
			case 255: 
				  switch (v) {
					  case 0: sendPacket(251,status); break;
					  case 1: sendPacket(254,crc_err); break;
					  case 2: status = 2; break;
					  case 3: sendPacket(251,SPI_osize); break;
					  case 4: sendPacket(251,SPI_isize); break;
					  case 5: sendPacket(251,loop_ms); break;
					  case 6: sendPacket(251,failsafe); break;
					  case 7: sendPacket(251,code); break;
					  case 254: break; //dummy - used for SPI queued message retrieval  
				  }
				  break;
			default: 
#ifdef DEBUG
				  byte c = packet[3];
				  Serial.print("Unknown command: "); Serial.print(t); Serial.print(" "); Serial.print(v); Serial.print(" "); Serial.println(c);
#endif
				  break;
		}
	}

}


#ifdef ALTHOLD

void log_altitude() {
	sendPacket(18,alt_hold_target);
	sendPacket(19,alt);
	sendPacket(20,vz);
}
#endif

void log_accel() {
	static float _accelMax[3] = {0.f,0.f,0.f};
	static float _accelMin[3] = {0.f,0.f,0.f};

	for (i=0;i<3;i++) {
		if (mympu.accel[i]<_accelMin[i]) _accelMin[i] = mympu.accel[i];
		if (mympu.accel[i]>_accelMax[i]) _accelMax[i] = mympu.accel[i];
	}

	if ((loop_count%20)==0) { //200Hz so 10times a sec... -> every 100ms
		for (i=0;i<3;i++) {
			sendPacket(12+i,_accelMax[i]*1000.f);
			_accelMax[i] = 0.f;
		}
	}
	else if ((loop_count%20)==10) { //200Hz so 10times a sec... -> every 100ms
		for (i=0;i<3;i++) {
			sendPacket(15+i,_accelMin[i]*1000.f);
			_accelMin[i] = 0.f;
		}
	}
}

void log_gyro() {
	for (i=0;i<3;i++)
		sendPacket(1+i,mympu.gyro[i]*100.f);
}

void log_ypr() {

	for (i=0;i<3;i++)
		sendPacket(4+i,mympu.ypr[i]*100.f);
	sendPacket(7,yaw_target*100.f);
}

void log_motor() {
	for (i=0;i<4;i++)
		sendPacket(8+i,motor[(motor_order >> (i*2)) & 0x3]);
}


void log_debug() {
	if (log_mode==1) log_accel(); 
	if (!(loop_count%25)) {
		//
		// Serial.println();
	}
}

void log() {
#ifdef DEBUG
	log_debug();
#else
	switch(log_mode) {
		case 0: break;

		case 1: 
			log_accel(); 
			break;

		case 2: 
			if ((loop_count%20)==0) //200Hz -> every 100ms
				log_gyro();
			else if ((loop_count%20)==10) //200Hz -> every 100ms
				log_motor();
			break;
		case 3:
			if ((loop_count%20)==0) //200Hz -> every 100ms
				log_ypr();
			else if ((loop_count%20)==10)
				log_motor();
			break;

#ifdef ALTHOLD
		case 4:
			if ((loop_count%20)==0)  //200Hz so 10times a sec... -> every 100ms
				log_altitude();
			break;
		case 5: 
			if ((loop_count%20)==0) //200Hz -> every 100ms
				log_ypr();
#ifdef ALTHOLD
			else if ((loop_count%20)==10) //200Hz -> every 100ms
				log_altitude();
#endif
			break;

#endif

		default: break;
	};
#endif
}

int8_t run_crash_detector() {
	static uint8_t crash_detector = 0;
	static float crash_alt;

	if (fly_mode!=0 || yprt[3]<=motor_pwm[1]) return 0; //dont do crash detection if not in stabilized mode

	if (abs(mympu.ypr[1])>=60.f || abs(mympu.ypr[2])>60.f ) crash_detector++;
	else crash_detector = 0;

	if (crash_detector==1) crash_alt = alt;
	else if (abs(alt-crash_alt) > 50.f)  //free fall? if we changed altitude by over 50cm
		crash_detector = 0;

	if (accel_crash_detector>0.f)
		if (abs(mympu.accel[0])>accel_crash_detector || abs(mympu.accel[1])>accel_crash_detector || abs(mympu.accel[2])>accel_crash_detector)
			crash_detector = 200; 

	if (crash_detector>=200) { //1sec
		status=251;
		motor_idle();
		return 1;
	}

	return 0;
}

void alt_override(float target, float climb) {
	vz_desired = climb;
	alt_hold_target = target;
	if (alt_hold) alt_hold = 2; //override only if we are already in alt_hold, otherwise do nothing
	else {
		failsafe = 0;
		yprt[3] = motor_pwm[0];
	}
	//this means failsafe will only work when in alt_hold
	//also, when failsafe is angaged, disengaging alt_hold will disengage failsafe too
	//this is to be able to regain control while in failsafe
}

#define DELAY_LAND_MS 2000 
#define LAND_SPEED 50.f //cm/s

int8_t run_failsafe() {
	static uint8_t land_detector;
	static float failsafe_alt;
	float land_speed;

	if (!failsafe) return 0;

#ifdef ALTHOLD
	if (failsafe==1) { 
		if  (yprt[3]>motor_pwm[2]) {  //ensure we are in flight (failsafe can be engaged from a different places)
			land_detector = 0;
			failsafe = 2;
			alt_hold = 1;
			failsafe_alt = alt;
			failsafeStart = millis();
			yprt[0] = yprt[1] = yprt[2] = 0;
			fly_mode = 0;
		} else {
			motor_idle();
		}
		return 0;
	} 

// TESTING
//	motor_idle();
//	return 0;
// TESTING
	if (millis()-failsafeStart>=max_failsafe_ms) {
		status = 252;
		motor_idle();
		return -1; 
	}

	if (millis()-failsafeStart<(unsigned long)DELAY_LAND_MS) return 0;

	if (vz>-30.f && yprt[3]<motor_pwm[2]) land_detector++;
	else land_detector = 0;

	if (land_detector >= 200) { //1sec grace period. LANDED
		motor_idle();
		status = 254;
		return 1;
	} 


	//perform landing
	land_speed = alt>500.f?(alt>1500.f?3*LAND_SPEED:2*LAND_SPEED):LAND_SPEED;

	failsafe_alt -= (land_speed * loop_s);
	alt_override(failsafe_alt, -land_speed);
#else
	motor_idle();
	failsafe = 0;
#endif
	return 0;
}

void run_althold() {
#ifdef ALTHOLD
	if (baro_counter>0) { //if there is no recent baro reading dont do alt_hold
		baro_counter--;
		//maintain altitude & velocity
		//when quadcopter goes up accel+, vz+, alt+;
		//when quadcopter goes down accel-, vz-, alt-;
		accel_z = constrain(mympu.accel[2],-1.f,1.f)*982.f; //convert accel to cm/s (9.82 * 100)
		//accel_z = mympu.accel[2]*982.f; //convert accel to cm/s (9.82 * 100)

		accel_corr += (alt_err * bc3 * loop_s);
		vz += (alt_err * bc2 * loop_s);
		alt_corr += (alt_err * bc1 * loop_s);
		vz_inc = (accel_z + accel_corr) * loop_s; //v = a * t
		alt_base += (vz + vz_inc * 0.5f) * loop_s; //s = v * t
		alt = alt_base + alt_corr;
		vz += vz_inc;
		buf_push(&alt_buf, alt_base);
		//end maintain altitude & velocity 

		// do altitude PID
		pos_err = alt_hold_target - alt;
		ld = MAX_ACCEL / (2.f * pid_alt.Kp * pid_alt.Kp);
		if (pos_err > 2.f*ld) 
			vz_target = sqrt(2.f * MAX_ACCEL * (pos_err-ld)); 	
		else if (pos_err < -2.f*ld)
			vz_target = -sqrt(2.f * MAX_ACCEL * (-pos_err-ld)); 	
		else {
			pid_update(&pid_alt,pos_err, loop_s);
			vz_target = pid_alt.value;
		}
		// end altitude PID

		if (alt_hold == 2) vz_target = vz_desired;
		// do velocity PID
		//alpha filter @ 4Hz
		vz_err += 0.111635f * (vz_target - vz - vz_err);
		pid_update(&pid_vz, vz_err, loop_s);
		// end velocity PID

		//alpha filter @ 2Hz
		accel_err += 0.059117f * (pid_vz.value - accel_z - accel_err);
		pid_update(&pid_accel,accel_err,loop_s);

		if (alt_hold) {
			yprt[3] = (int)(motor_pwm[3] + pid_accel.value); 
		} else {
			pid_reset(&pid_alt);
			pid_reset(&pid_vz);
			pid_reset(&pid_accel);
			accel_err = 0.f;
			vz_err = 0.f;
		} 
	} else {
		if (alt_hold) code = 2;
		alt_hold = 0; //baro expired
	}
#endif
}

void run_pid() {
	//if (abs(mympu.ypr[2])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling excessivly
	//if (abs(mympu.ypr[1])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if pitching excessivly 
	//flip recovery end

	//do STAB PID                                                            


	//if ((yaw_target-mympu.ypr[0])<-180.0f) yaw_target*=-1.f; 
	//if ((yaw_target-mympu.ypr[0])>180.0f) yaw_target*=-1.f; 

	//yaw pids
	if (abs(yprt[0])>YAW_THRESHOLD) { 
		pid_s[0].value = pid_acro_p*(yprt[0]-YAW_THRESHOLD*(yprt[0]/abs(yprt[0])));
		yaw_target = mympu.ypr[0];
	}
	else pid_update(&pid_s[0],wrap_180(yaw_target-mympu.ypr[0]),loop_s); 
	//yaw pids end

	if (fly_mode == 0) { //STAB
		for (i=1;i<3;i++)                                               
			pid_update(&pid_s[i],yprt[i]-mympu.ypr[i],loop_s);

	} else if (fly_mode == 1) { //RATE
		for (i=1;i<3;i++)                                               
			pid_update(&pid_s[i],yprt[i]*pid_acro_p,loop_s);
	} 

	//do RATE PID                                                            
	for (i=0;i<3;i++) {                                                  
		pid_update(&pid_r[i],pid_s[i].value-mympu.gyro[i],loop_s);
	}                                                                        

}

void controller_loop() {
#ifdef MPU9150
	ret = mympu_update_compass();
	if (ret < 0) {
#ifdef DEBUG
		Serial.print("Error reading compass: "); Serial.println(ret);
#endif
	}
#endif
	ret = mympu_update();
	if (ret != 0) {
		if (ret==1) return; //no packet available
		mpu_err++;
#ifdef DEBUG
		Serial.print("mympu_update: "); Serial.println(ret);
#endif
		if (mpu_err>10) {
			motor_idle();
			status = 253;
			code = ret;
		}
		return;	
	} else mpu_err = 0;
#ifdef DEBUG
	switch (ret) {
		case 0: c++; break;
		case 1: np++; return; 
		case 2: err_o++; return;
		case 3: err_c++; return;
	}

#endif
	if (++loop_count==200)
		loop_count = 0;

	loop_ms = millis() - p_millis;
	loop_s = (float)(loop_ms)/1000.f;
	p_millis = millis();
#ifdef DEBUG
		if (loop_ms>50) 
#else
		if (loop_ms>10) 
#endif
		{
			code = 1;
			if (!failsafe) failsafe = 1;
		}

	run_althold(); //need to run this before failsafe to set throttle 

	if (run_crash_detector()) return;

	if (run_failsafe()) return;

	run_pid();

	//calculate motor speeds                                        
	motor[motor_order & 0x3] = (int)(yprt[3]+pid_r[2].value-pid_r[1].value+pid_r[0].value);
	motor[(motor_order >> 2) & 0x3] = (int)(yprt[3]+pid_r[2].value+pid_r[1].value-pid_r[0].value);
	motor[(motor_order >> 4) & 0x3] = (int)(yprt[3]-pid_r[2].value-pid_r[1].value-pid_r[0].value);
	motor[(motor_order >> 6) & 0x3] = (int)(yprt[3]-pid_r[2].value+pid_r[1].value+pid_r[0].value);

	log();

	if (yprt[3] < motor_pwm[1]) {
		motor_idle();
		yaw_target = mympu.ypr[0];
		for (i=0;i<3;i++) {                                              
			pid_reset(&pid_r[i]);
			pid_reset(&pid_s[i]);
		}                                                                    
		return;
	}

	for (i=0;i<4;i++) 
		motor[i] = (motor[i]<motor_pwm[1])?motor_pwm[1]:motor[i];

	writeMotors();
}

int8_t gyroCal() {
#define CALIBRATION_LOOPS 1600 //8sec * 200
#define GRAVITY_LOOPS 200 
#define FAILED_LOOPS 4000 //20sec
#define CALIBRATION_THRESHOLD 3.0f

	static float accel = 0.0f;
	static unsigned int loop_c = 0;

	ret = mympu_update();
	if (ret!=0) {
		if (ret==1) return -1;
                mpu_err++;
#ifdef DEBUG
		Serial.print("MPU error! "); Serial.println(ret);
#endif
		if (mpu_err>10) {
			status = 253;
			code = ret;
		}
		return -1;
	} else mpu_err = 0;

	if (mympu.gyro[0]>-CALIBRATION_THRESHOLD && mympu.gyro[1]>-CALIBRATION_THRESHOLD && mympu.gyro[2]>-CALIBRATION_THRESHOLD &&    
			mympu.gyro[0]<CALIBRATION_THRESHOLD && mympu.gyro[1]<CALIBRATION_THRESHOLD && mympu.gyro[2]<CALIBRATION_THRESHOLD) { 
		loop_c++;
		if (loop_c>CALIBRATION_LOOPS) accel += mympu.accel[2]; 
	} else 
		if (loop_c>CALIBRATION_LOOPS) status = 255; //if not in CALIBRATION_THRESHOLD but were there before 
		else loop_c = 0;

	if (loop_c>=(CALIBRATION_LOOPS+GRAVITY_LOOPS)) { 
		mympu.gravity = accel / GRAVITY_LOOPS;
		return 0;
	}

	return -1;
}

long statusMillis = 0l;
void loop() {
	
	if (millis() - statusMillis > 1000) {
		sendStatus();
		statusMillis = millis();
	}

	if (status==0) {
		delay(100);	
		initAVR();
		status = 1;
	}

	process_command();

	//status = 2 set by client after sending config 

	switch (status) {
		case 2: 
#ifdef DEBUG
			ret = mympu_open(mpu_addr,50,gyro_orientation);
#else
			ret = mympu_open(mpu_addr,200,gyro_orientation);
#endif
			delay(150); //without this it the behaviour is very flaky (sometimes will not boot)
//			sendPacket(253,ret); 
			if (ret == 0) { 
				status = 3;
				mympu_reset_fifo();
			} else {
				Fastwire::reset();
				delay(25);
				code = ret;
			}
			break;
		case 3:
			initMotors();
			status = 4;
			break;
		case 4:
			if (gyroCal()==0) 
				status = 5;
			p_millis = millis()-1; //to ensure the first run has dt_ms of 5ms
			last_command = p_millis;
			break;

		case 5:
			controller_loop();
			break;
		
		case 251: case 252: case 253: case 254: 
			deinitMotors(); //very useful for some ESCs if you loose you quadcopter in high grass
			break;

		default: break;
	}
}

