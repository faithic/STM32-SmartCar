#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
}

/**
  * @brief  毫秒计数器，由 TIM4 的 20ms 中断累加
  * @note   为什么不用 SysTick：本文件的 Delay_us() 每次都直接改写 SysTick
  *         的 LOAD/VAL/CTRL 寄存器，写 CTRL 时 bit1(TICKINT)=0，会把 SysTick
  *         中断关掉而且再也不会打开。所以"1ms 中断累加 tick"那套在这里跑不起来。
  *         TIM1/TIM2/TIM3 被编码器和 PWM 占了，TIM4 是唯一空闲的周期中断源。
  * @retval 无
  */
static volatile uint32_t Delay_TickMs = 0;

/**
  * @brief  给毫秒计数器加 20ms，在 TIM4 的更新中断里调用
  * @param  无
  * @retval 无
  */
void Delay_TickUpdate(void)
{
	Delay_TickMs += 20;
}

/**
  * @brief  取上电以来的毫秒数
  * @note   本车上的分辨率是 20ms（见上面 Delay_TickMs 的说明）。用来给主循环
  *         做分频计时，比如"每 500ms 打印一次"，比 Delay_ms(500) 好——后者会把
  *         主循环（包括 BT_control()）阻塞半秒。
  * @param  无
  * @retval 上电以来的毫秒数
  */
uint32_t GetTick(void)
{
	return Delay_TickMs;
}
