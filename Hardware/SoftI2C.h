/**
  ******************************************************************************
  * @file    SoftI2C.h
  * @brief   软件 I2C（位翻转）驱动 —— MPU6050 / VL53L1X 共用
  *          引脚：PA4 = SDA，PA5 = SCL
  ******************************************************************************
  */

#ifndef _SOFTI2C_H_
#define _SOFTI2C_H_

#include "stm32f10x.h"

// 引脚定义
// 注意：PA4/PA5 不是 5V 容忍引脚（PB8/PB9 才是），模块的 VCC 必须接 3.3V，
// 否则板载上拉会把 SDA/SCL 拉到 5V，超过 PA4/PA5 的 VDD+0.3V 上限
#define IIC_PORT      GPIOA
#define IIC_SDA_PIN   GPIO_Pin_4
#define IIC_SCL_PIN   GPIO_Pin_5

//
// @简介：初始化软件 I2C 的两根引脚（开漏输出）并把总线置为空闲
//
void IIC_Init(void);

//
// @简介：总线恢复 + 上电自检（必须在 IIC_Init 之后调用一次）
// @返回值：0  - 两线都能读回高，总线正常
//          -1 - SDA 被拉住（查接线/上拉/从机供电）
//          -2 - SCL 被拉住
//
int IIC_BusRecover(void);

//
// @简介：写从机的连续寄存器
// @参数 dev7    - 从机地址（7 位形式，MPU6050 填 0x68）
// @参数 reg     - 起始寄存器地址
// @参数 pData   - 要写入的数据
// @参数 Size    - 数据字节数
// @返回值：0 - 成功，-1 - 寻址失败，-2 - 数据被拒收（约定同 my_lib/i2c.h）
//
int IIC_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *pData, uint16_t Size);

//
// @简介：读从机的连续寄存器（写寄存器地址 -> 重复起始 -> 连读，中间不产生 STOP）
// @参数 dev7    - 从机地址（7 位形式，MPU6050 填 0x68）
// @参数 reg     - 起始寄存器地址
// @参数 pBuffer - 接收缓冲区
// @参数 Size    - 要读取的字节数
// @返回值：0 - 成功，-1 - 寻址失败，-2 - 数据被拒收
//
int IIC_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *pBuffer, uint16_t Size);

#endif
