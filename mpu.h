#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

#define wrap_180(x) (x<-180.f?x+360.f:(x>180.f?x-360.f:x))
#define shift_180(x) (x+180.f>180.f?x-180.f:x+180.f)

struct s_mympu {
	float ypr[3];
	float gyro[3];
	float accel[3];
#ifdef MPU9150
    float comp[3];
#endif
    float gravity;
};

extern struct s_mympu mympu;

int8_t mympu_open(short addr,unsigned int rate,unsigned short orient);
int8_t mympu_update();
void mympu_reset_fifo();
#ifdef MPU9150
int8_t mympu_update_compass();
#endif

#endif

