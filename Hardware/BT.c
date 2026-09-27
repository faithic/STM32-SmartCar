#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "OLED.h"
#include "Car.h"
uint8_t Serial_RxData;		
uint8_t Serial_RxFlag;		
uint8_t RxData;
void Serial_Init(void)
{
	//开启时钟  USART3,GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	//初始化GPIO引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	//初始化USART
	USART_InitTypeDef USART_InitStructure;					
	USART_InitStructure.USART_BaudRate = 9600;//波特比				
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//数据发送和接收
	USART_InitStructure.USART_Parity = USART_Parity_No;	//无校验位	
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位长度为1
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//数据位长度为8	
	USART_Init(USART3, &USART_InitStructure);				
	//配置中断路口
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			
	//选择NVIC中断分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			
	//配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;					
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
	NVIC_Init(&NVIC_InitStructure);							
	//SUART使能
	USART_Cmd(USART3, ENABLE);								
}

//发送一个字节
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);		
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
}

//发送一组字节
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		
	{
		Serial_SendByte(Array[i]);		
	}
}
//发送字符串
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);		
	}
}
//次方函数
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	
	while (Y --)			
	{
		Result *= X;		
	}
	return Result;
}
//发送数字
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	
	}
}
//串口接收标志位 -> 读取标志位状态后置0
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			
	{
		Serial_RxFlag = 0;
		return 1;					
	}
	return 0;						
}
//接收数据
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;			
}
//USART3中断函数
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)	//判断是否进行中断	
	{
		Serial_RxData = USART_ReceiveData(USART3);			//存放数据
		Serial_RxFlag = 1;									
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);		//清除标志位								
	}
}
//整体逻辑：有数据要接收->进入中断Serial_RxFlag置1，并获取Serial_RxData值
//->进入蓝牙控制函数，经过Serial_GetRxFlag()==1的判断后，Serial_RxFlag置0，然后根据数据执行操作
//蓝牙控制函数
void BT_control(void)
{
     if(Serial_GetRxFlag()==1)            //通过if函数进入功能
        {
            RxData=Serial_GetRxData();    
			if(RxData==0x40)
			{
//				OLED_ShowString(1,1,"前进");
				Car_Move(50,50);
				//由于Car.c中相关速度还未配置，控制的函数先不写
			}
			if(RxData==0x41)
			{    
//				OLED_ShowString(1,1,"后退");
				Car_Move(-50,-50);             
			}
			if(RxData==0x42)
			{
//				OLED_ShowString(1,1,"左转");
				Car_Move(30,80); 
			}		
			if(RxData==0x43)
			{
//				OLED_ShowString(1,1,"右转");
				Car_Move(80,30); 
			}			
			if(RxData==0x44)
			{    
//				OLED_ShowString(1,1,"停止");
				Car_Stop();
			}	
			if(RxData==0x45)
			{
//				OLED_ShowString(1,1,"右自转");
				Car_Move(50,-50);
			}
			if(RxData==0x46)
			{
//				OLED_ShowString(1,1,"左自转");
				Car_Move(-50,50);
			}
			if(RxData==0x47)
			{
//				OLED_ShowString(1,1,"避碍");
				
			}
			if(RxData==0x48)
			{    
//				OLED_ShowString(1,1,"循迹");
				
			}
		}
}