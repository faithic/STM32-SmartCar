#include "stm32f10x.h"                  // Device header
//用于在PB0,PB1引脚输出PWM波形，CCR值可改  通道3->PB0->PWMB 通道4->PB1->PWMA
//注意：PB0是TIM3的通道3 PB1是TIM3的通道4
void PWM_Init(void)
{
	//打开时钟 TIM3 GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//初始化GPIO引脚   配置PB0，PB1引脚为复用推挽输出 输出PWM波  
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP ;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//选择内部时钟源
	TIM_InternalClockConfig(TIM3);
	//配置时基单元           频率:72000000/100/7200=100Hz
	TIM_TimeBaseInitTypeDef TIM_TimeBaseinitStructure;
	TIM_TimeBaseinitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseinitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseinitStructure.TIM_Period = 100-1;   //ARR
	TIM_TimeBaseinitStructure.TIM_Prescaler = 72-1;   //PSC
	TIM_TimeBaseinitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseinitStructure);
	//配置输出比较通道
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//为结构体赋初始值
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置TIM3的通道口3和通道口4
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择PWM模式1 向上计数时：CNT < CCR → 有效电平；CNT ≥ CCR → 无效电平。
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//高电平为有效电平即电平不翻转
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;  //CCR  占空比：CCR/(ARR+1)
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	//时钟使能
	TIM_Cmd(TIM3,ENABLE);
}
//改变通道3占空比
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare3(TIM3,Compare);//通过改变通道3的CCR的值改变占空比
}
//改变通道4占空比
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare4(TIM3,Compare);
}