#include "stm32f10x.h"                  // Device header
void Timer_Init(void)
{
	//开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			
	//配置时钟源
	TIM_InternalClockConfig(TIM4);		
	//时基单元初始化
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;				
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;				
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);	
	//频率：72000000/720/2000=50Hz ->0.02s  -->这里可以改进入中断的时间 每隔0.02s进入一次中断
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);						//清除定时器更新标志位	
    //中断输出使能
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);					
	//NVIC 分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				
	//配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;						
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			
	NVIC_Init(&NVIC_InitStructure);								
	//TIM使能
	TIM_Cmd(TIM4, ENABLE);			
}
