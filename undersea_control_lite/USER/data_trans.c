#include "data_trans.h"

uint16_t Control_Data[CONTROL_FRAME_LENGTH];
uint8_t Custom_Data[CUSTOM_FRAME_LENGTH];
uint16_t t;
//uint8_t Relay_Data;

void AutoReceive_Init(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除TIMx的中断待处理位:TIM 中断源
	TIM_TimeBaseInitStructure.TIM_Period = 100;//设置自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 35999;//设置用来作为TIMx时钟频率预分频值，100Khz计数频率
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);	
	TIM_Cmd(TIM2,ENABLE); //使能或者失能TIMx外设
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );	//使能或者失能指定的TIM中
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //打开TIM5_IRQn的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;  //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能
	NVIC_Init(&NVIC_InitStructure);	
}

void ConvertReceDataToReg(void){
	uint8_t pos,offset,chksum=0;
	for(pos=0;pos<USART2_DMA_Rece_Len;pos++){
		if(USART2_DMA_Rece_Buf[pos]==FRAME_START){
			for(offset=0;offset<FRAME_LENGTH-1;offset++){
				chksum+=USART2_DMA_Rece_Buf[pos+offset];
			}
			if(chksum==USART2_DMA_Rece_Buf[pos+FRAME_LENGTH-1]){
				Control_Data[0]=USART2_DMA_Rece_Buf[pos+1]*256+USART2_DMA_Rece_Buf[pos+2];
				Control_Data[1]=USART2_DMA_Rece_Buf[pos+3]*256+USART2_DMA_Rece_Buf[pos+4];
				Control_Data[2]=USART2_DMA_Rece_Buf[pos+5]*256+USART2_DMA_Rece_Buf[pos+6];
				Control_Data[3]=USART2_DMA_Rece_Buf[pos+7]*256+USART2_DMA_Rece_Buf[pos+8];
				Control_Data[4]=USART2_DMA_Rece_Buf[pos+9]*256+USART2_DMA_Rece_Buf[pos+10];
				Control_Data[5]=USART2_DMA_Rece_Buf[pos+11]*256+USART2_DMA_Rece_Buf[pos+12];
				Control_Data[6]=USART2_DMA_Rece_Buf[pos+13]*256+USART2_DMA_Rece_Buf[pos+14];
				Control_Data[7]=USART2_DMA_Rece_Buf[pos+15]*256+USART2_DMA_Rece_Buf[pos+16];
				Control_Data[8]=USART2_DMA_Rece_Buf[pos+17]*256+USART2_DMA_Rece_Buf[pos+18];
				Control_Data[9]=USART2_DMA_Rece_Buf[pos+19]*256+USART2_DMA_Rece_Buf[pos+20];
				Control_Data[10]=USART2_DMA_Rece_Buf[pos+21]*256+USART2_DMA_Rece_Buf[pos+22];
				Control_Data[11]=USART2_DMA_Rece_Buf[pos+23]*256+USART2_DMA_Rece_Buf[pos+24];
				Control_Data[12]=USART2_DMA_Rece_Buf[pos+25]*256+USART2_DMA_Rece_Buf[pos+26];
				Control_Data[13]=USART2_DMA_Rece_Buf[pos+27]*256+USART2_DMA_Rece_Buf[pos+28];
				Control_Data[14]=USART2_DMA_Rece_Buf[pos+29]*256+USART2_DMA_Rece_Buf[pos+30];
				Control_Data[15]=USART2_DMA_Rece_Buf[pos+31]*256+USART2_DMA_Rece_Buf[pos+32];
				//Relay_Data=USART2_DMA_Rece_Buf[pos+33];
				Custom_Data[0]=USART2_DMA_Rece_Buf[pos+34];
				Custom_Data[1]=USART2_DMA_Rece_Buf[pos+35];
				Custom_Data[2]=USART2_DMA_Rece_Buf[pos+36];
				Custom_Data[3]=USART2_DMA_Rece_Buf[pos+37];
				Custom_Data[4]=USART2_DMA_Rece_Buf[pos+38];
				Custom_Data[5]=USART2_DMA_Rece_Buf[pos+39];
				Custom_Data[6]=USART2_DMA_Rece_Buf[pos+40];
				Custom_Data[7]=USART2_DMA_Rece_Buf[pos+41];
				Custom_Data[8]=USART2_DMA_Rece_Buf[pos+42];
				Custom_Data[9]=USART2_DMA_Rece_Buf[pos+43];
				Custom_Data[10]=USART2_DMA_Rece_Buf[pos+44];
				Custom_Data[11]=USART2_DMA_Rece_Buf[pos+45];
				Custom_Data[12]=USART2_DMA_Rece_Buf[pos+46];
				goto finishsearch;
			}
		}
	}
	finishsearch:
	for(pos=0;pos<USART2_DMA_Rece_Len;pos++)USART2_DMA_Rece_Buf[pos]=0;
}

void TIM2_IRQHandler(){
	static uint8_t i;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		i++;
		
		ConvertReceDataToReg();
		USART2_DMA_SendByte(t/100);
		if(i==20){
			i=0;
			GPIOB->ODR^=GPIO_Pin_0;
			t=roll_avg(Get_Temprate(),0);
		}
	}	
}
