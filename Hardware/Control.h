#ifndef _CONTROL_H
#define _CONTROL_H
#include "stm32f10x.h"   
#define CONTROL_OUTPUT_MAX     100.0f
#define TARGET_SPEED_MAX     100.0f
//1.管理控制器状态
void Control_Start(void);
void Control_Stop(void);
//2.管理整个控制器的目标速度
void Control_SetTarget(float Left, float Right);
//3.限幅函数
float Control_Limit(float Value,float Max);
//4.完成一次控制周期  利用编码器反馈，计算新的PWM输出，使实际速度不断逼近目标速度
void Control_Update(void);
#endif
