#ifndef __SOFT_I2C__
#define __SOFT_I2C__
#include "sys.h"

#define I2C_GPIO_GROUP GPIOB
#define SDA_IN()  {I2C_GPIO_GROUP->CRH&=0XFFFF0FFF;I2C_GPIO_GROUP->CRH|=0X00008000;}
#define SDA_OUT() {I2C_GPIO_GROUP->CRH&=0XFFFF0FFF;I2C_GPIO_GROUP->CRH|=0X00003000;}

//IO操作函数	 
#define IIC_SCL    PBout(10) 		//SCL
#define IIC_SDA    PBout(11) 		//SDA	 
#define READ_SDA   PBin(11) 	 		//输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

uint8_t I2C_ArrayRead(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *buff,uint8_t length);
void I2C_CmdWrite(uint8_t SlaveAddress,uint8_t command);
uint16_t I2C_ByteRead16(uint8_t SlaveAddress,uint8_t REG_Address);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);


#endif
