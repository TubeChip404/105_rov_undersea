#ifndef __USART_DMA__
#define __USART_DMA__

#include "sys.h"
#include "delay.h"
#include "stdio.h"
#include "adc.h"

#define USART1_DMA_Rece_Len 96
#define USART1_DMA_Send_Len 48

#define USART3_DMA_Rece_Len 96
#define USART3_DMA_Send_Len 48

void uart_init(u32 bound);
void USART1_DMA_Send(void);
void USART1_DMA_SendByte(uint8_t data);
void USART1_DMA_SendBytes(uint8_t length);

void USART3_DMA_Send(void);
void USART3_DMA_SendByte(uint8_t data);
void USART3_DMA_SendBytes(uint8_t length);

extern uint8_t USART1_DMA_Rece_Buf[USART1_DMA_Rece_Len];
extern uint8_t USART1_DMA_Send_Buf[USART1_DMA_Send_Len];

extern uint8_t USART3_DMA_Rece_Buf[USART3_DMA_Rece_Len];
extern uint8_t USART3_DMA_Send_Buf[USART3_DMA_Send_Len];
#endif

