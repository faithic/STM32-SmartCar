#include "stm32f10x.h"                  // Device header
#include "Delay.h"
//Echo 高电平持续的时间，反映超声波往返传播时间
#define ULTRASONIC_TIMEOUT_US   30000
#define ULTRASONIC_TIMEOUT_TICKS (30000UL * 72UL)
void HCSR04_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    // Trig PB8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Echo PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}
//触发信号 -> 10微秒的高电平
void HCSR04_Trigger(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_us(10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}
//计时开始(启动SysTick)
void HCSR04_SysTickStart(void)
{
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005;
}
//计时结束
void HCSR04_SysTickStop(void)
{
    SysTick->CTRL = 0x00000004;
}
//
uint32_t HCSR04_SysTickGetElapsed(void)
{
    return 0xFFFFFF - SysTick->VAL;
}
//等待置高电平
uint8_t HCSR04_WaitEchoHigh(void)
{
    while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == RESET)
    {
        if(HCSR04_SysTickGetElapsed() >= ULTRASONIC_TIMEOUT_TICKS)
        {
            return 0;
        }
    }
    return 1;
}
//等待置低电平
uint8_t Ultrasonic_WaitEchoLow(void)
{
    while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == SET)
    {
        if(HCSR04_SysTickGetElapsed() >= ULTRASONIC_TIMEOUT_TICKS)
        {
            return 0;
        }
    }
    return 1;
}
//返回距离
float Ultrasonic_GetDistance(void)
{
    uint32_t Count;
    float Time_us;
    float Distance;
    // 发送Trig
    HCSR04_Trigger();
    // 启动SysTick
    HCSR04_SysTickStart();
    // 等待Echo变高
    if(HCSR04_WaitEchoHigh() == 0)
    {
        HCSR04_SysTickStop();
        return -1.0f;
    }
    // Echo变高，从这里开始正式计时
    SysTick->VAL = 0;
    // 等待Echo变低
    if(Ultrasonic_WaitEchoLow() == 0)
    {
        HCSR04_SysTickStop();
        return -1.0f;
    }
    // Echo变低，读取时间
    Count = HCSR04_SysTickGetElapsed();
    // 停止SysTick
    HCSR04_SysTickStop();
    // 时钟数 → μs
    Time_us = (float)Count / 72.0f;
    // μs → cm
    Distance = Time_us / 58.0f;
    return Distance;
}
/*    模块测试代码
int main(void)
{
    OLED_Init();
    Ultrasonic_Init();

    while(1)
    {
        float Distance;

        Distance = Ultrasonic_GetDistance();

        if(Distance < 0)
        {
            OLED_ShowString(1,1,"ERROR");
        }
        else
        {
            OLED_ShowNum(1,1,(uint16_t)Distance,3);
            OLED_ShowString(1,5,"cm");
        }

        Delay_ms(100);
    }
}
*/