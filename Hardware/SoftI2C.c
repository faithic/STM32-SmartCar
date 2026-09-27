/**
  ******************************************************************************
  * @file    SoftI2C.c
  * @brief   软件 I2C（位翻转）驱动 —— MPU6050 / VL53L1X 共用
  *          引脚：PA4 = SDA，PA5 = SCL
  ******************************************************************************
  */

#include "SoftI2C.h"

// ============================================================================
// 时序延时
// ============================================================================

//
// @简介：软件 I2C 的位延时（约 2us，对应 SCL 约 200kHz）
//
// @备注：刻意不用 Delay_us()，理由见小车工程手册附录 B 坑 8：小车工程的 Delay.c 和
//        HCSR04.c 在抢 SysTick，而 App_MPU6050_UpdateAngle() 要在 20ms 定时器中断里跑，
//        那里只能靠空循环延时。
//
//        i 必须是 volatile：否则 -O1 以上编译器会把这个"没有副作用的空循环"整个删掉，
//        每比特只剩几次 GPIO 写，SCL 会冲到 1MHz 以上，I2C 立刻失效。
//        本工程当前编译等级是 -O0（uvprojx 里 <Optim>1</Optim>），不删也能跑；
//        但搬进小车工程后优化等级会变，所以这里现在就写成优化安全的形式。
//
static void IIC_Delay(void)
{
	volatile uint8_t i;
	for (i = 0; i < 20; i++);
}

// ============================================================================
// 引脚底层
// ============================================================================

static void SCL(uint8_t v)
{
	if (v) GPIO_SetBits(IIC_PORT, IIC_SCL_PIN);
	else   GPIO_ResetBits(IIC_PORT, IIC_SCL_PIN);
}

static void SDA(uint8_t v)
{
	if (v) GPIO_SetBits(IIC_PORT, IIC_SDA_PIN);
	else   GPIO_ResetBits(IIC_PORT, IIC_SDA_PIN);
}

//
// @简介：读 SDA 的引脚电平
// @备注：开漏输出模式下写 1 = 释放总线（由上拉拉高），写 0 = 主动拉低；而 STM32F103
//        在任何输出模式下输入施密特触发器都使能，IDR 读到的就是引脚真实电平。
//        所以一根 SDA 既能写又能读，不需要来回切输入/输出模式。
//        唯一的例外是 GPIO_Mode_AIN：模拟输入下施密特触发器关闭，IDR 恒读 0。
//        移植到小车工程时要注意 Hardware/ADC_DMA.c 会把 PA2~PA7 配成 AIN。
//
static uint8_t SDA_Read(void)
{
	return (GPIO_ReadInputDataBit(IIC_PORT, IIC_SDA_PIN) == Bit_SET) ? 1 : 0;
}

static uint8_t SCL_Read(void)
{
	return (GPIO_ReadInputDataBit(IIC_PORT, IIC_SCL_PIN) == Bit_SET) ? 1 : 0;
}

// ============================================================================
// 协议基本动作
// ============================================================================

//
// @简介：初始化软件 I2C
//
void IIC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.GPIO_Pin   = IIC_SCL_PIN | IIC_SDA_PIN;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;  // 开漏输出：写1=释放总线，写0=拉低
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_PORT, &GPIO_InitStruct);

	// 两条线都释放 = 总线空闲
	SCL(1);
	SDA(1);
}

//
// @简介：起始位。也可以直接当重复起始用：进入时 SCL 已被上一个字节的收尾拉低、
//        SDA 已释放，所以 SCL 抬高的那一刻 SDA 还是高电平，再拉低就是合法的 RESTART
//
static void IIC_Start(void)
{
	SDA(1); SCL(1); IIC_Delay();
	SDA(0); IIC_Delay();   // SCL 高时 SDA 下降 = 起始
	SCL(0); IIC_Delay();   // 拉低时钟，准备传位
}

//
// @简介：停止位
//
static void IIC_Stop(void)
{
	SDA(0); IIC_Delay();
	SCL(1); IIC_Delay();
	SDA(1); IIC_Delay();   // SCL 高时 SDA 上升 = 停止
}

//
// @简介：发送一个字节，并读回从机的应答
// @返回值：0 - 从机应答（ACK），非 0 - 无应答（NAK）
//
static uint8_t IIC_SendByte(uint8_t Byte)
{
	uint8_t i, Ack;

	for (i = 0; i < 8; i++)
	{
		SCL(0);
		SDA((Byte & 0x80) ? 1 : 0);   // 先把数据摆好，再抬时钟，从机在时钟高时采样
		IIC_Delay();
		SCL(1);
		IIC_Delay();
		Byte <<= 1;
	}

	// 第 9 个时钟：主机释放 SDA，看从机拉不拉低
	SCL(0); SDA(1); IIC_Delay();
	SCL(1); IIC_Delay();
	Ack = SDA_Read();
	SCL(0);                            // 收尾把时钟拉低，和 IIC_ReadByte 的出口状态一致

	return Ack;
}

//
// @简介：接收一个字节
// @参数 Ack：1 - 回 ACK（后面还要继续读），0 - 回 NACK（读完了）
// @返回值：读到的字节
// @备注：主机的应答位也要自己产生第 9 个时钟才能送达从机。少了那一拍，从机看到
//        的永远是 NAK——单字节读还能凑合（后面紧跟 STOP），连续读就会从第二个字节
//        起全部读回 0xFF，而且是静默的
//
static uint8_t IIC_ReadByte(uint8_t Ack)
{
	uint8_t i, Byte = 0;

	SDA(1);   // 先释放 SDA，交给从机驱动

	for (i = 0; i < 8; i++)
	{
		SCL(0); IIC_Delay();
		SCL(1); IIC_Delay();   // 时钟高，读走一位
		Byte <<= 1;
		if (SDA_Read()) Byte |= 0x01;
	}

	// 第 9 个时钟：主机驱动应答位，并自己把时钟打出去
	SCL(0);
	SDA(Ack ? 0 : 1);   // 0 = 拉低 = ACK
	IIC_Delay();
	SCL(1);
	IIC_Delay();
	SCL(0); SDA(1); IIC_Delay();   // 释放，准备下一字节或 STOP

	return Byte;
}

// ============================================================================
// 寄存器级接口
// ============================================================================

//
// @简介：写从机的连续寄存器
// @返回值：0 - 成功，-1 - 寻址失败，-2 - 数据被拒收
//
int IIC_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *pData, uint16_t Size)
{
	uint16_t i;

	IIC_Start();

	// 从机地址 + 写
	if (IIC_SendByte((uint8_t)(dev7 << 1)) != 0)
	{
		IIC_Stop();
		return -1;
	}

	// 寄存器地址
	if (IIC_SendByte(reg) != 0)
	{
		IIC_Stop();
		return -2;
	}

	for (i = 0; i < Size; i++)
	{
		if (IIC_SendByte(pData[i]) != 0)
		{
			IIC_Stop();
			return -2;
		}
	}

	IIC_Stop();

	return 0;
}

//
// @简介：读从机的连续寄存器
// @返回值：0 - 成功，-1 - 寻址失败，-2 - 数据被拒收
//
int IIC_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *pBuffer, uint16_t Size)
{
	uint16_t i;

	if (Size == 0) return 0;

	IIC_Start();

	// 从机地址 + 写
	if (IIC_SendByte((uint8_t)(dev7 << 1)) != 0)
	{
		IIC_Stop();
		return -1;
	}

	// 寄存器地址
	if (IIC_SendByte(reg) != 0)
	{
		IIC_Stop();
		return -2;
	}

	// 重复起始（中间不产生 STOP），换成读方向
	IIC_Start();

	if (IIC_SendByte((uint8_t)((dev7 << 1) | 0x01)) != 0)
	{
		IIC_Stop();
		return -1;
	}

	// 前 Size-1 个字节回 ACK（还要继续读），最后一个回 NACK（告诉从机读完了）
	for (i = 0; i < Size; i++)
	{
		pBuffer[i] = IIC_ReadByte((i == Size - 1) ? 0 : 1);
	}

	IIC_Stop();

	return 0;
}

//
// @简介：总线恢复 + 上电自检
// @返回值：0 - 两线都能读回高，总线正常；-1 - SDA 被拉住；-2 - SCL 被拉住
//
// @备注：上一次运行的固件如果在 I2C 传输中途被复位（典型场景：只按了 MCU 的复位键，
//        没给 MPU6050 断电），从机可能还拽着 SDA 不放。
//        硬件 I2C 这时会卡在 BUSY 位上，现象很显眼；软件 I2C 不会卡死，但更阴——
//        主机写 1 时 SDA 仍是低，从机读到的位全错，可它照样会回 ACK，于是初始化
//        一路打印"成功"，实际一个寄存器都没配上。所以这一步不是可选的。
//
int IIC_BusRecover(void)
{
	uint8_t i;

	// #1. 主机先把两条线都释放
	SDA(1);
	SCL(1);
	IIC_Delay();

	// #2. 从机若拽住 SDA，最多敲 9 个时钟把它顶出来（一个字节 8 位 + 1 个应答位，
	//     敲满 9 拍从机一定会走到释放 SDA 的那一拍）
	for (i = 0; i < 9; i++)
	{
		if (SDA_Read()) break;   // SDA 已经释放，不用再敲

		SCL(0); IIC_Delay();
		SCL(1); IIC_Delay();
	}

	// #3. 补一个 STOP，让从机的状态机复位到空闲
	IIC_Stop();

	// #4. 自检：空闲时两条线都该被上拉拉高。读到 0 说明线被拽住，或者根本没接上拉
	//     （模块没插好/没供电/板载上拉失效）
	if (!SDA_Read()) return -1;
	if (!SCL_Read()) return -2;

	return 0;
}
