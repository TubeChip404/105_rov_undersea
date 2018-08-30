#ifndef __DATA_TRANS__
#define __DATA_TRANS__

#include "usart_dma.h"
#include "sys.h"
#include "adc.h"

#define NULL_DATA 	 	0x00
#define CONTROL_DATA 	0x01
#define RECEIVE_DATA 	0x02
#define CONFIG_DATA  	0x03

#define FRAME_START 	0xAA
#define FRAME_LENGTH 	48
#define CONTROL_FRAME_LENGTH	16
#define CUSTOM_FRAME_LENGTH 	13

extern uint16_t Control_Data[CONTROL_FRAME_LENGTH];
extern uint8_t Custom_Data[CUSTOM_FRAME_LENGTH];
//extern uint8_t Relay_Data;

void AutoReceive_Init(void);

#endif

