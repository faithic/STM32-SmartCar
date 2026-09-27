#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f10x.h"
#include "PWM.h"
void Motor_Init(void);
//设置左轮速度
void Motor_SetLeftSpeed(int16_t PWM);
//设置右轮速度
void Motor_SetRightSpeed(int16_t PWM);
//设置车的速度
void Motor_SetSpeed(int16_t PWM1,int16_t PWM2);
#endif
