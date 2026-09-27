#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP ;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2);
	
}
void LED1_ON (void)//LED1点亮函数
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}
void LED1_OFF (void)//LED1熄灭函数
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
}
void LED2_ON (void)//LED2点亮函数
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}
void LED2_OFF (void)//LED2熄灭函数
{
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
}
void LED1_Turn(void)
{
	//   函数GPIO_ReadOutputDataBit()用于读取指定的输出端口的电平，低电平为0，高电平为1
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1) == 0)//对应的引脚PA1为低电平，即LED1处于发光状态
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_1);//使LED1熄灭
	}
	else//对应引脚为高电平
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);//使LED1亮
	}
}
void LED2_Turn(void)
{
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_2) == 0)//对应的引脚为低电平，即LED2处于发光状态
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_2);
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	}
}
