#include "stm32f10x.h"                  // Device header
//由于舵机模块中用来输出PWM波的时钟为TIM4，该时钟已经用于定时中断，所以在这里无需重复配置
//注意：该模块的初始化应在TIM4的初始化后面
void Servo_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//初始化GPIO引脚  PB6
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP ;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//为结构体赋初始值
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置TIM4的通道口1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 150;  //CCR ->初始90度
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
}
//改变占空比
void Servo_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM4,Compare);
}
// 设置舵机角度
void Servo_SetAngle(float Angle)
{
    Servo_SetCompare1((uint16_t)(Angle / 180.0f * 200.0f + 50.0f));
}
// 0.5ms / 20ms ->  50/2000(ARR)
// 1ms / 20ms -> 100/2000(ARR)
// 1.5ms / 20ms -> 150/2000(ARR)