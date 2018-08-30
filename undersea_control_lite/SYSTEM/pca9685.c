#include "pca9685.h"

void PCA9685_Set_Freq(float iFreq){
	uint8_t i;
	float prescaleval=25000000.0;
	uint8_t prescale=0;
	uint8_t oldmode=0,newmode=0;
	iFreq=iFreq*0.9f;
	prescaleval /= 4096.0;
	prescaleval /= iFreq;
	prescaleval -= 1.0;
	prescale=(uint8_t)prescaleval;
	IIC_Write_One_Byte(PCA9685_ADDR,0x0,0);
	oldmode=IIC_Read_One_Byte(PCA9685_ADDR,0x0);
	newmode=(oldmode&0x7f)|0x10;
	IIC_Write_One_Byte(PCA9685_ADDR,0x0,newmode);
	IIC_Write_One_Byte(PCA9685_ADDR,0xfe,prescale);
	IIC_Write_One_Byte(PCA9685_ADDR,0x0,oldmode);
	delay_ms(100);
	IIC_Write_One_Byte(PCA9685_ADDR,0x0,oldmode | 0xa1);
	for(i=0;i<16;i++){
		IIC_Write_One_Byte(PCA9685_ADDR,0x6+4*i,0x0);
		IIC_Write_One_Byte(PCA9685_ADDR,0x7+4*i,0x0);
	}
}

void PCA9685_Set_Pwm(uint8_t iPin,uint16_t iPwm){
	IIC_Write_One_Byte(PCA9685_ADDR,0x6+4*iPin,0x0);
	IIC_Write_One_Byte(PCA9685_ADDR,0x7+4*iPin,0x0);
	IIC_Write_One_Byte(PCA9685_ADDR,0x8+4*iPin,iPwm&0xff);
	IIC_Write_One_Byte(PCA9685_ADDR,0x9+4*iPin,iPwm>>8);
}
