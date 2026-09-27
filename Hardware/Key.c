#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void Key_Init(void)//初始化
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU ;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
}
uint8_t Key_Getnum(void)//该函数返回Key_Getnum的值，用于判断按键1，2是否被按下
{
	uint8_t Key_Getnum = 0;//先给Key_Getnum赋初值
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0)//检测按键是否被按下  按键被按下，引脚——低电平，函数的返回值是0
	{
		Delay_ms (20);//消抖延时
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0)//循环等待按键松开 
		Delay_ms (20);
		Key_Getnum =1;
	}
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)//检测按键是否被按下
	{
		Delay_ms (20);//消抖延时
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)//循环等待按键松开
		Delay_ms (20);
		Key_Getnum =2;
	}
	return Key_Getnum ;
	
	
	
}