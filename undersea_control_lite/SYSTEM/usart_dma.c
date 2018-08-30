#include "usart_dma.h"
//初始化IO 串口1 
//bound:波特率
#ifdef USART1_ENALBE
	uint8_t USART1_DMA_Rece_Buf[USART1_DMA_Rece_Len];
	uint8_t USART1_DMA_Send_Buf[USART1_DMA_Send_Len];
#endif

#ifdef USART3_ENALBE
	uint8_t USART3_DMA_Rece_Buf[USART3_DMA_Rece_Len];
	uint8_t USART3_DMA_Send_Buf[USART3_DMA_Send_Len];
#endif

#ifdef USART2_ENALBE
	uint8_t USART2_DMA_Rece_Buf[USART2_DMA_Rece_Len];
	uint8_t USART2_DMA_Send_Buf[USART2_DMA_Send_Len];
#endif

void uart_init(u32 bound){
    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD, ENABLE); //使能USART1，GPIOA时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3|RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef USART1_ENALBE
	USART_DeInit(USART1);  //复位串口1
   //USART1_TX   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX  A.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
	#endif
	
	#ifdef USART3_ENALBE
	USART_DeInit(USART3);  //复位串口3
   //USART3_TX   PD.8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOD, &GPIO_InitStructure); //初始化PA9
   
    //USART3_RX  PD.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOD, &GPIO_InitStructure);  //初始化PA10
	
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
	#endif
	
	#ifdef USART2_ENALBE
	USART_DeInit(USART2);  //复位串口2
   //USART1_TX   PA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX  A.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
	#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Usart1 NVIC 配置
	#ifdef USART1_ENALBE
  NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
	#endif
	
	#ifdef USART3_ENALBE
	//Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
  
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
	#endif
	
	#ifdef USART2_ENALBE
	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
	#endif
	
	
   //USART1 初始化设置
	 #ifdef USART1_ENALBE
  USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1,USART_IT_TC,DISABLE);//使能串口空闲中断
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//失能接收中断
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//使能发送中断

  USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//使能串口发送DMA
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能串口接收DMA
  USART_Cmd(USART1, ENABLE);
	#endif
	
	//USART3 初始化设置
	#ifdef USART3_ENALBE
  USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3,USART_IT_TC,DISABLE);//使能串口空闲中断
  USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);//失能接收中断
  USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//使能发送中断

  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);//使能串口发送DMA
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//使能串口接收DMA
  USART_Cmd(USART3, ENABLE);
	#endif
	
	//USART2 初始化设置
  #ifdef USART2_ENALBE
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式

  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2,USART_IT_TC,DISABLE);//使能串口空闲中断
  USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);//失能接收中断
  USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//使能发送中断

  USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);//使能串口发送DMA
  USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//使能串口接收DMA
  USART_Cmd(USART2, ENABLE);
	#endif
  
  //usart1相应的DMA配置
	#ifdef USART1_ENALBE
	DMA_DeInit(DMA1_Channel5);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_DMA_Rece_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART1_DMA_Rece_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel5, ENABLE);  //正式驱动DMA传输
	
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_DMA_Send_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART1_DMA_Send_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	
  //DMA_Cmd(DMA1_Channel4, ENABLE);  //正式驱动DMA传输
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	#endif
	
	#ifdef USART3_ENALBE
	//usart2相应的DMA配置
  DMA_DeInit(DMA1_Channel3);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART3_DMA_Rece_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART3_DMA_Rece_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel3, ENABLE);  //正式驱动DMA传输
	
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART3_DMA_Send_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART3_DMA_Send_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	
  //DMA_Cmd(DMA1_Channel4, ENABLE);  //正式驱动DMA传输
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	#endif
	
	#ifdef USART2_ENALBE
	//usart1相应的DMA配置
  DMA_DeInit(DMA1_Channel6);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_DMA_Rece_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART2_DMA_Rece_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel6, ENABLE);  //正式驱动DMA传输
	
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_DMA_Send_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = USART2_DMA_Send_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	
  //DMA_Cmd(DMA1_Channel4, ENABLE);  //正式驱动DMA传输
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
	#endif
}
#ifdef USART1_ENALBE
void USART1_DMA_Send(void){
  DMA_SetCurrDataCounter(DMA1_Channel4, USART1_DMA_Send_Len);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

void USART1_DMA_SendByte(uint8_t data){
	USART1_DMA_Send_Buf[0]=data;
	DMA_SetCurrDataCounter(DMA1_Channel4, 1);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

void USART1_DMA_SendBytes(uint8_t length){
	DMA_SetCurrDataCounter(DMA1_Channel4, length);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

//串口中断函数
void USART1_IRQHandler(void){
//	uint8_t i;
	 if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET){
		 DMA_Cmd(DMA1_Channel5,DISABLE);
		 USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		 USART_ReceiveData(USART1);
		 DMA_SetCurrDataCounter(DMA1_Channel5,USART1_DMA_Rece_Len); 
		 DMA_Cmd(DMA1_Channel5,ENABLE);
	 }
}

//串口1DMA方式发送中断  
void DMA1_Channel4_IRQHandler(void){  
    DMA_ClearFlag(DMA1_FLAG_TC4);  
    DMA_Cmd(DMA1_Channel4,DISABLE);  
}  
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USART3_ENALBE
void USART3_DMA_Send(void){
  DMA_SetCurrDataCounter(DMA1_Channel2, USART3_DMA_Send_Len);
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

void USART3_DMA_SendByte(uint8_t data){
	USART3_DMA_Send_Buf[0]=data;
	DMA_SetCurrDataCounter(DMA1_Channel2, 1);
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

void USART3_DMA_SendBytes(uint8_t length){
	DMA_SetCurrDataCounter(DMA1_Channel2, length);
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

//串口中断函数
void USART3_IRQHandler(void){
//	uint8_t i;
	 if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET){
		 DMA_Cmd(DMA1_Channel3,DISABLE);
		 USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		 USART_ReceiveData(USART3);
		 DMA_SetCurrDataCounter(DMA1_Channel3,USART3_DMA_Rece_Len); 
		 DMA_Cmd(DMA1_Channel3,ENABLE);
	 }
}

//串口1DMA方式发送中断  
void DMA1_Channel2_IRQHandler(void){  
    DMA_ClearFlag(DMA1_FLAG_TC2);  
    DMA_Cmd(DMA1_Channel2,DISABLE);  
} 
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USART2_ENALBE
void USART2_DMA_Send(void){
  DMA_SetCurrDataCounter(DMA1_Channel7, USART2_DMA_Send_Len);
  DMA_Cmd(DMA1_Channel7, ENABLE);
}

void USART2_DMA_SendByte(uint8_t data){
	USART2_DMA_Send_Buf[0]=data;
	DMA_SetCurrDataCounter(DMA1_Channel7, 1);
  DMA_Cmd(DMA1_Channel7, ENABLE);
}

void USART2_DMA_SendBytes(uint8_t length){
	DMA_SetCurrDataCounter(DMA1_Channel7, length);
  DMA_Cmd(DMA1_Channel7, ENABLE);
}

//串口中断函数
void USART2_IRQHandler(void){
//	uint8_t i;
	 if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET){
		 DMA_Cmd(DMA1_Channel6,DISABLE);
		 USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		 USART_ReceiveData(USART2);
		 DMA_SetCurrDataCounter(DMA1_Channel6,USART2_DMA_Rece_Len); 
		 DMA_Cmd(DMA1_Channel6,ENABLE);
	 }
}

//串口1DMA方式发送中断  
void DMA1_Channel7_IRQHandler(void){  
    DMA_ClearFlag(DMA1_FLAG_TC7);  
    DMA_Cmd(DMA1_Channel7,DISABLE);  
} 
#endif
