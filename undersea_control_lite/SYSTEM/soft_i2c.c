#include "soft_i2c.h"
#include "delay.h"

void IIC_Init(void){					     
 	RCC->APB2ENR|=1<<3;									 
	I2C_GPIO_GROUP->CRH&=0XFFFF00FF;	
	I2C_GPIO_GROUP->CRH|=0X00003300;	   
	I2C_GPIO_GROUP->ODR|=3<<10;
}

void IIC_Start(void){
	SDA_OUT();     
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;
	delay_us(4);
	IIC_SCL=0;
}	  

void IIC_Stop(void){
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;
	delay_us(4);							   	
}

u8 IIC_Wait_Ack(void){
	u8 ucErrTime=0;
	SDA_IN();
	IIC_SDA=1;delay_us(4);	   
	IIC_SCL=1;delay_us(4);	 
	while(READ_SDA){
		ucErrTime++;
		if(ucErrTime>250)		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;  
	return 0;  
} 
void IIC_Ack(void){
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(4);
	IIC_SCL=1;
	delay_us(4);
	IIC_SCL=0;
}	    
void IIC_NAck(void){
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(4);
	IIC_SCL=1;
	delay_us(4);
	IIC_SCL=0;
}		  
void IIC_Send_Byte(u8 txd){                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;
    for(t=0;t<8;t++){              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(4);
		IIC_SCL=1;
		delay_us(4); 
		IIC_SCL=0;	
		delay_us(4);
    }	 
} 	     
u8 IIC_Read_Byte(unsigned char ack){
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ ){
        IIC_SCL=0; 
        delay_us(4);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(4); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack(); 
    return receive;
}

void I2C_CmdWrite(uint8_t SlaveAddress,uint8_t command){
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(command);
	IIC_Wait_Ack();
	IIC_Stop();
}

uint8_t I2C_ArrayRead(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *buff,uint8_t length){
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+1);
	IIC_Wait_Ack();
	while(length){
		if(length==1)*buff=IIC_Read_Byte(0);
			else *buff=IIC_Read_Byte(1);
		buff++;
		length--;
	}
	IIC_Stop();
	return 1;
}

uint16_t I2C_ByteRead16(uint8_t SlaveAddress,uint8_t REG_Address){
	uint8_t Read_REG1,Read_REG2;
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+1);
	IIC_Wait_Ack();
	Read_REG1=IIC_Read_Byte(1);
	Read_REG2=IIC_Read_Byte(1);
	IIC_Stop();
	return (Read_REG1<<8)|Read_REG2;
}

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data){
	IIC_Start();
	IIC_Send_Byte(daddr);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();
}
u8 IIC_Read_One_Byte(u8 daddr,u8 addr){
	uint8_t Read_REG1;
	IIC_Start();
	IIC_Send_Byte(daddr);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(daddr+1);
	IIC_Wait_Ack();
	Read_REG1=IIC_Read_Byte(0);
	IIC_Stop();
	return Read_REG1;
}
