#include "sys.h"
#include "usart_dma.h"
#include "delay.h"
#include "data_trans.h"
#include "soft_i2c.h"
#include "pca9685.h"
#include "adc.h"



int main(){
	uint8_t i;
	SystemInit();
	delay_init();
	uart_init(115200);
	IIC_Init();
	Adc_Init();
	ADC_TempSensorVrefintCmd(ENABLE);
	PCA9685_Set_Freq(50.0);
	PCA9685_Set_Pwm(0,2047);
	AutoReceive_Init();

	while(1){

		GPIOA->ODR^=GPIO_Pin_12;
		for(i=0;i<16;i++){
			PCA9685_Set_Pwm(i,Control_Data[i]);
		}
	}
}
