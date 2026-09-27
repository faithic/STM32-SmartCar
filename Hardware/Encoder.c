#include "stm32f10x.h"                  // Device header
volatile int16_t LeftCount;
volatile int16_t RightCount;
//左轮的编码器初始化函数
void Encoder1_Init(void)
{
	//开启时钟  GPIOA TIM1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							
	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);             
	//配置输入捕获通道
	TIM_ICInitTypeDef TIM_ICInitStructure;							
	TIM_ICStructInit(&TIM_ICInitStructure);							//结构体初始化，若结构体没有完整赋值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM1, &TIM_ICInitStructure);							
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;				
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM1, &TIM_ICInitStructure);							
	//配置编码器接口
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	//配置编码器模式以及两个输入通道是否反相
	//注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
	//TIM使能
	TIM_Cmd(TIM1, ENABLE);
}
//读取CNT的值，并清零
int16_t Encoder1_GetCount(void)
{
	int16_t Count;
	Count =(int64_t)TIM_GetCounter(TIM1);
	TIM_SetCounter(TIM1, 0);
	return Count;
}
//右轮的编码器初始化函数
void Encoder2_Init(void)
{
	//开启时钟  GPIOA TIM2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							
	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             
	//配置输入捕获通道
	TIM_ICInitTypeDef TIM_ICInitStructure;							
	TIM_ICStructInit(&TIM_ICInitStructure);							//结构体初始化，若结构体没有完整赋值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM2, &TIM_ICInitStructure);							
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;				
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM2, &TIM_ICInitStructure);							
	//配置编码器接口
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	//配置编码器模式以及两个输入通道是否反相
	//注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
	//TIM使能
	TIM_Cmd(TIM2, ENABLE);
}
//读取CNT的值，并清零
int16_t Encoder2_GetCount(void)
{
	int16_t Count;
	Count = (int16_t)TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM2, 0);
	return Count;
}
//直接初始化左右轮编码器
void Encoder_Init(void)
{
    Encoder1_Init();
    Encoder2_Init();
}
//返回左轮20ms采样周期内的编码器计数
int16_t Encoder_GetLeftCount(void)
{
    return LeftCount;
}
//返回右轮20ms采样周期内的编码器计数
int16_t Encoder_GetRightCount(void)
{
    return RightCount;
}