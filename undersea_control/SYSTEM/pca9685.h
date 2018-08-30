#ifndef __PCA9685_H__
#define __PCA9685_H__

#include "sys.h"
#include "delay.h"
#include "soft_i2c.h"

#define PCA9685_ADDR 0x80
#define PCA9685_FREQ 50.0

void PCA9685_Set_Freq(float iFreq);
void PCA9685_Set_Pwm(uint8_t iPin,uint16_t iPwm);


#endif
