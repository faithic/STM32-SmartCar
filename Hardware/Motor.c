#include "stm32f10x.h"                  // Device header
#include "PWM.h"
//用于配置对应引脚，通过调用函数，改变小车速度
//通道3->PB0->PWMB->右轮(对应PWM_SetCompare1)控制正反转 PB14，PB15    通道4->PB1->PWMA->左轮(对应PWM_SetCompare2)控制正反转 PB12,PB13
void Motor_Init(void)
{
	//初始化PWM
	PWM_Init();
	//开启时钟 GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//初始化GPIO引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP ;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
//设置左轮速度
void Motor_SetLeftSpeed(int16_t PWM)
{
	//限幅
	if(PWM>100)
	{
		PWM=100;
	}
	else if(PWM<-100)
	{
		PWM=-100;
	}
	if(PWM>0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		PWM_SetCompare2(PWM);
	}
	else if(PWM==0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		GPIO_SetBits(GPIOB,GPIO_Pin_13);
		PWM_SetCompare2(PWM);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		GPIO_SetBits(GPIOB,GPIO_Pin_13);
		PWM_SetCompare2(-PWM);
	}
}
//设置右轮速度
void Motor_SetRightSpeed(int16_t PWM)
{
	//限幅
	if(PWM>100)
	{
		PWM=100;
	}
	else if(PWM<-100)
	{
		PWM=-100;
	}
	if(PWM>0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		PWM_SetCompare1(PWM);
	}
	else if(PWM==0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		PWM_SetCompare1(PWM);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
		PWM_SetCompare1(-PWM);
	}
}
//设置车的速度
void Motor_SetSpeed(int16_t PWM1,int16_t PWM2)
{
	Motor_SetLeftSpeed(PWM1);
	Motor_SetRightSpeed(PWM2);
}