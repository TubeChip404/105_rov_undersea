#ifndef __USART_DMA__
#define __USART_DMA__

#include "sys.h"
#include "delay.h"
#include "stdio.h"

//#define USART1_ENALBE
#define USART2_ENALBE
//#define USART3_ENALBE

#ifdef USART1_ENALBE
	#define USART1_DMA_Rece_Len 96
	#define USART1_DMA_Send_Len 48
#endif

#ifdef USART3_ENALBE
	#define USART3_DMA_Rece_Len 96
	#define USART3_DMA_Send_Len 48
#endif

#ifdef USART2_ENALBE
	#define USART2_DMA_Rece_Len 96
	#define USART2_DMA_Send_Len 48
#endif

void uart_init(u32 bound);

#ifdef USART1_ENALBE
	void USART1_DMA_Send(void);
	void USART1_DMA_SendByte(uint8_t data);
	void USART1_DMA_SendBytes(uint8_t length);
#endif

#ifdef USART3_ENALBE
	void USART3_DMA_Send(void);
	void USART3_DMA_SendByte(uint8_t data);
	void USART3_DMA_SendBytes(uint8_t length);
#endif

#ifdef USART2_ENALBE
	void USART2_DMA_Send(void);
	void USART2_DMA_SendByte(uint8_t data);
	void USART2_DMA_SendBytes(uint8_t length);
#endif

#ifdef USART1_ENALBE
	extern uint8_t USART1_DMA_Rece_Buf[USART1_DMA_Rece_Len];
	extern uint8_t USART1_DMA_Send_Buf[USART1_DMA_Send_Len];
#endif

#ifdef USART3_ENALBE
	extern uint8_t USART3_DMA_Rece_Buf[USART3_DMA_Rece_Len];
	extern uint8_t USART3_DMA_Send_Buf[USART3_DMA_Send_Len];
#endif

#ifdef USART2_ENALBE
	extern uint8_t USART2_DMA_Rece_Buf[USART2_DMA_Rece_Len];
	extern uint8_t USART2_DMA_Send_Buf[USART2_DMA_Send_Len];
#endif
#endif

